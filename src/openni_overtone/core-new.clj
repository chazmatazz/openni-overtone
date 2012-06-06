(ns openni-overtone.core
  (:require [bifocals.core :as bifocals]
            [quil.core :as quil]
            [overtone.live :as overtone]
            [clojure.math.numeric-tower :as math]))

;; what makes a human
(def joint-pairs
  [[:head :neck] [:neck :left-shoulder] [:neck :right-shoulder]
   [:left-shoulder :left-elbow] [:right-shoulder :right-elbow]
   [:left-elbow :left-hand] [:right-elbow :right-hand]
   [:left-shoulder :torso] [:right-shoulder :torso]
   [:torso :left-hip] [:torso :right-hip]
   [:left-hip :left-knee] [:right-hip :right-knee]
   [:left-knee :left-foot] [:right-knee :right-foot]])

(def hands [:left-hand :right-hand])
(def feet [:left-foot :right-foot])

(def stage
  {:min {:x -400, :y -800, :z 0},
   :max {:x 400, :y 2000, :z 5000},
   :center {:x 0, :y 0, :z 2500}})

;; the size of an axis of the stage
(def stage-size
  (into {}
        (for [axis [:x :y :z]]
          [axis (- (axis (:max stage)) (axis (:min stage)))])))

(def hist-size 60)

;; skeletons map
;; each entry in the map is a uid to skeleton-hist circular buffer
;; buffer must be decircularized when necessary
(def skeletons-hist (atom {}))

(defn get-buf-head [frame-count]
  (- hist-size (+ (mod frame-count hist-size) 1)))

(defn get-skel-hist [buf-head circular-buffer]
  (concat
   (take-last (- hist-size buf-head) circular-buffer)
   (take buf-head circular-buffer)))

;; is the skeleton hist ready
(defn ready [skeleton-hist]
  (and
   (= (count skeleton-hist) hist-size)
   (not (reduce (fn [acc v] (or acc v)) false (map nil? skeleton-hist)))))

;; convert uids to keywords
(defn keywordize [skeletons]
  (into {} (for [[k v] (seq skeletons)] [(keyword (str k)) v])))

;; ------------------------
;; skeleton history helpers
;; note: now only calculates for ctl-joints and ctl-axis
;; ------------------------

(def ctl-joints (cons :head (concat hands feet)))

(def ctl-axes [:x :y])

;; derivatives

;; given 2 skeletons, calculate the derivative
(defn skeleton-derive [new-skel old-skel]
  (into {} (for [joint ctl-joints]
             [joint (into {} (for [axis ctl-axes]
                        [axis (-
                               (axis (joint new-skel))
                               (axis (joint old-skel)))]))])))

;; given a derivate function and a list, calculate the derivative list
(defn derivative [f lst]
  (into [] (for [i (range (- (count lst) 1))]
             (f (nth lst i) (nth lst (+ i 1))))))

;; just an alias
(defn skeleton-derivative [lst]
  (derivative skeleton-derive lst))

(defn velocity [skeleton-hist]
  (skeleton-derivative skeleton-hist))

(def fps 30)

(defn setup []
  (quil/smooth)
  ;; This function connects to the kinect. You must call it first. Otherwise,
  ;; there will be NPEs.
  (bifocals/setup)
  (quil/frame-rate fps))

(defn draw-line
  [p1 p2]
  (quil/line (:x p1) (:y p1) (:x p2) (:y p2)))

(def frame-count (atom 0))

(def screen-modes [:debug :off :performance])

(def screen-mode (atom :off))

(defn draw []
  (quil/background 0)
  ;; You must call this in the draw function. Otherwise, your depth image will be
  ;; all black, and user/skeleton tracking will not work.
  (bifocals/tick)
  ;; triggers drawing and sound
  (swap! frame-count inc))

(quil/defsketch kinect
  :title "Unfolding Perception"
  :setup setup
  :draw draw
  ; If your depth image seems truncated, or the window is much larger than it,
  ; check its dimensions by calling `bifocals/depth-width` and
  ; `bifocals/depth-height`, and then adjust the sketch size accordingly.
  :size [1280 960])

;; ----------------------
;; instrument definitions
;; ----------------------

;; the defratios aren't working with definst in Overtone 0.6.0

(overtone/definst inst-sin-third
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc [freq (* freq 5/4)])))

(overtone/definst inst-sin-fourth
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc [freq (* freq 4/3)])))

(overtone/definst inst-sin-osc
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc freq)))

;; uid-insts
(def keyword-uid-insts (atom {}))

;; control volume in sound booth
(def master-vol 1)

(def fudge 10)

(def inst-toggles-feet (merge (into {} (for [joint (cons :head hands)] [joint 0]))
                              (into {} (for [joint feet] [joint 1]))))

(def inst-toggles-hands (merge (into {} (for [joint (cons :head feet)] [joint 0]))
                               (into {} (for [joint hands] [joint 1]))))

(def inst-toggles-all (merge (into {} (for [joint ctl-joints] [joint 1]))))

(def inst-toggles {:feet inst-toggles-feet,
                   :hands inst-toggles-hands,
                   :all inst-toggles-all})

(def inst-vol-map {:head 0.15,
                   :left-foot 0.3,
                   :right-foot 0.4,
                   :left-hand 0.075,
                   :right-hand 0.075})

(defn get-inst-vols [ct]
  (let [itv (ct inst-toggles)]
    (let [s (apply + (into [] (for [[k v] inst-vol-map] (* (k itv) v))))]
      (into {} (for [[k v] inst-vol-map] [k (/ (* (k itv) v) s)])))))

(def joint-y-stage-range {:head [-1000 500],
                    :left-hand [-500 500],
                    :right-hand [-500 500],
                    :left-foot [-1000 500],
                    :right-foot [-1000 500]})

(def velocity-frac 0.1)

(def joint-freq-range {:head [60 250],
                       :left-foot [120 240],
                       :right-foot [30 120]})

(def joint-chords {:left-hand (overtone/chord :C3 #{0 4 7 12}),
                   :right-hand (overtone/chord :F3 :major7)})

;; for changes during performance
;; one of [:feet, :hands, :all]
(def ctl-type (atom :feet))

(def ctl-hands-every 10)

;; reset the insts whilst hacking on code
(defn reset-insts []
  (overtone/stop)
  (reset! keyword-uid-insts {}))

(defn sixteenth-graph [start-x start-y vec abs-max]
  (let [w (* (quil/width) 1/4)
        h (* (quil/height) 1/4)
        mid-y (+ start-y (* h 1/2))
        s (count vec)
        stride (/ w s)
        h-max (/ h 2)
        offset (/ stride 2)]
    (doseq [i (range s)]
      (let [x (+ start-x (+ offset (* i stride)))]
        (quil/line x mid-y x
                   (+ mid-y (overtone/scale-range
                             (nth vec i 0)
                             (- 0 abs-max) abs-max
                             (- 0 h-max) h-max)))))))

;; when frame-count changes
(defn on-frame [the-key the-ref old-frame-count new-frame-count]
  ;; create new skeletons-hist
  ;; advancing pointer so that circular buffers read left-to-right
  (let [keyword-uid-skeletons (keywordize @bifocals/skeletons)]

    ;; draw skeletons
    (when-not (= @screen-mode :off)
      (when (= @screen-mode :performance) (quil/scale 2))
      (quil/image (bifocals/depth-image) 0 0)
      (quil/stroke-weight 3)
      (quil/stroke 255 0 180)
      (doseq [[keyword-uid skel] keyword-uid-skeletons]
              (let [project-skeleton (bifocals/project-skeleton skel)]
                (doseq [joint-pair joint-pairs]
                  (draw-line
                   ((first joint-pair) project-skeleton)
                   ((second joint-pair) project-skeleton))))))

    ;; add new instruments
    (doseq [keyword-uid (keys keyword-uid-skeletons)]
      (when (not (contains? @keyword-uid-insts keyword-uid))
        (let [insts {:head (inst-sin-osc),
                     :left-hand (inst-sin-third),
                     :right-hand (inst-sin-third),
                     :left-foot (inst-sin-fourth),
                     :right-foot (inst-sin-fourth)}]
          (swap! keyword-uid-insts assoc keyword-uid insts))))

    ;; make noise, update skel-hist
    (let [buf-head (get-buf-head new-frame-count)
          new-skeletons-hist
          (into {} (for [[keyword-uid skel] keyword-uid-skeletons]
                     [keyword-uid
                      (into
                       []
                       (for [i (range hist-size)]
                         (if (= i buf-head)
                           skel
                           (nth (keyword-uid @skeletons-hist) i nil))))]))
          keyword-uids (keys new-skeletons-hist)
          set-keyword-uids (set keyword-uids)
          num-uids (count keyword-uids)]
      (let [num-joints (count ctl-joints)
            time-delta (/ 1000 fps num-joints)]
        (doseq [[keyword-uid insts] @keyword-uid-insts]
          (if (contains? set-keyword-uids keyword-uid)
            (let [circular-buffer (keyword-uid new-skeletons-hist)
                  skel-hist (get-skel-hist buf-head circular-buffer)]
              (if (ready skel-hist)
                (doseq [i (range num-joints)]
                  (let [joint (nth ctl-joints i)
                        inst-id (joint insts)
                        joint-vol (joint (get-inst-vols @ctl-type))]
                    (if (= 0 joint-vol)
                      (overtone/ctl inst-id :vol 0)
                      (let [y-stage-range (joint joint-y-stage-range)
                            min-pos-y (first y-stage-range)
                            max-pos-y (second y-stage-range)
                            abs-max-pos-y (max (math/abs min-pos-y)
                                               (math/abs max-pos-y))
                            pos-vec-y
                            (into []
                                  (for [skel skel-hist] (:y (joint skel))))
                            pos-sum-y (apply + pos-vec-y)
                            f-scale
                            (partial overtone/scale-range
                                     pos-sum-y
                                     (* min-pos-y (count pos-vec-y))
                                     (* max-pos-y (count pos-vec-y)))
                            freq
                            (if (contains? (set hands) joint)
                              (let [chord (joint joint-chords)]
                                (overtone/midi->hz
                                 (overtone/note
                                  (nth chord
                                       (int (f-scale
                                             0
                                             (- (count chord) 1)))))))
                              (let [freq-range (joint joint-freq-range)]
                                (f-scale
                                 (first freq-range)
                                 (second freq-range))))
                            abs-max-vel-x
                            (* velocity-frac (:x stage-size))
                            vel-vec-x
                            (map (fn [vel-x] (overtone/scale-range
                                             vel-x
                                             (- 0 abs-max-vel-x) abs-max-vel-x
                                             (- 0 abs-max-vel-x) abs-max-vel-x))
                                 (into []
                                (for [v-skel (velocity skel-hist)]
                                  (:y (joint v-skel)))))
                            vel-sum-x (apply + vel-vec-x)]
                        (quil/stroke-weight 1)
                        (sixteenth-graph (+ i (* (quil/width) 1/2))
                                         (* (quil/height) 1/8)
                                         [vel-sum-x]
                                         (* abs-max-vel-x (count vel-vec-x)))
                        (sixteenth-graph (+ i (* (quil/width) 3/4))
                                         (* (quil/height) 1/8)
                                         vel-vec-x
                                         abs-max-vel-x)
                        (when (or (not (contains? (set hands) joint))
                                  (= 0 (mod new-frame-count ctl-hands-every)))
                          (overtone/at
                           (+ (overtone/now) (* i time-delta))
                           (overtone/ctl
                            inst-id
                            :freq
                            freq
                            :vol
                            (overtone/scale-range
                             (math/abs vel-sum-x)
                             0 (* abs-max-vel-x (count vel-vec-x))
                             0 (/ (* fudge master-vol joint-vol) num-uids)))))))))))
            (doseq [[keyword-uid inst] insts] (overtone/ctl inst :vol 0)))))
      (reset! skeletons-hist new-skeletons-hist))))

(add-watch frame-count :on-frame on-frame)

;; for instrument hacking
;; (reset-insts)

;; initial state: (= @screen-mode :off), (= @ctl-type :feet)

;; for debugging
;; (reset! screen-mode :debug)

;; display during performance
;; (reset! screen-mode :performance)

;; sound during performance
;; (reset! ctl-type :hands)
;; (reset! ctl-type :all)
;; (reset! ctl-type :feet)

;; @screen-mode
;; @ctl-type
