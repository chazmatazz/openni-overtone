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

(def hist-size 10)

;; skeletons map
;; each entry in the map is a uid to skeleton-hist circular buffer
(def skeletons-hist (atom {}))

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
;; note: now only calculates for ctl-joints
;; ------------------------

(def ctl-joints (cons :head (concat hands feet)))

;; derivatives

;; given 2 skeletons, calculate the derivative
(defn skeleton-derive [new-skel old-skel]
  (into {} (for [joint ctl-joints]
             [joint (into {} (for [axis [:x :y :z]]
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

(defn acceleration [skeleton-hist]
  (skeleton-derivative (skeleton-derivative skeleton-hist)))

;; given a skeleton history
;; calculates the range for each joint as {:min min, :max max}
(defn skeleton-range [skeleton-hist]
  (into {}
        (for [joint ctl-joints]
          [joint
           (into
            {}
            (for [axis [:x :y :z]]
              [axis {:min
                     (reduce
                      (fn [acc h]
                        (let [v (axis (joint h))]
                          (if (< acc v) v acc)))
                      overtone/INF skeleton-hist),
                     :max
                     (reduce
                      (fn [acc h]
                        (let [v (axis (joint h))]
                          (if (> acc v) v acc)))
                      overtone/INF skeleton-hist)
                     :sum
                     (reduce
                      (fn [acc h]
                        (let [v (axis (joint h))]
                          (+ acc v)))
                      0 skeleton-hist)}]))])))

(defn setup []
  (quil/smooth)
  ;; This function connects to the kinect. You must call it first. Otherwise,
  ;; there will be NPEs.
  (bifocals/setup)
  (quil/frame-rate 30))

(defn draw-line
  [p1 p2]
  (quil/line (:x p1) (:y p1) (:x p2) (:y p2)))

(def frame-count (atom 0))

(defn draw []
  (quil/background 0)
  ;; You must call this in the draw function. Otherwise, your depth image will be
  ;; all black, and user/skeleton tracking will not work.
  (bifocals/tick)
  (quil/image (bifocals/depth-image) 0 0)
  (quil/stroke-weight 3)
  (quil/stroke 255 0 180)
  (doseq [[keyword-uid skeleton-hist] @skeletons-hist]
    (let [i (mod (- @frame-count 1) hist-size)
          skel (if (> (count skeleton-hist) i)
                 (nth skeleton-hist i)
                 nil)]
      (when-not (nil? skel)
        (let [project-skeleton
              (bifocals/project-skeleton skel)]
          (doseq [joint-pair joint-pairs]
            (draw-line
             ((first joint-pair) project-skeleton)
             ((second joint-pair) project-skeleton)))))))
  (swap! frame-count inc))

(quil/defsketch kinect
  :title "Unfolding Perception"
  :setup setup
  :draw draw
  ; If your depth image seems truncated, or the window is much larger than it,
  ; check its dimensions by calling `bifocals/depth-width` and
  ; `bifocals/depth-height`, and then adjust the sketch size accordingly.
  :size [640 480])

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

(overtone/definst inst-pulse-fifth
  [freq 440
   vol 0]
  (* vol (overtone/pulse [freq (* freq 3/2)])))

(overtone/definst inst-sin-osc
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc freq)))

;; uid-insts
(def keyword-uid-insts (atom {}))

;; when quil/frame-count changes
(defn update-skeletons-hist [the-key the-ref old-frame-count new-frame-count]
    (let [keyword-uid-skeletons (keywordize @bifocals/skeletons)]
    ;; add new instruments
    (doseq [keyword-uid (keys keyword-uid-skeletons)]
      (when (not (contains? @keyword-uid-insts keyword-uid))
        (let [insts {:head (inst-sin-osc),
                     :left-hand (inst-sin-third),
                     :right-hand (inst-pulse-fifth),
                     :left-foot (inst-sin-fourth),
                     :right-foot (inst-sin-fourth)}]
          (swap! keyword-uid-insts assoc keyword-uid insts))))
    ;; add to skeletons history
    (swap!
     skeletons-hist
     (fn [s-h]
       (into {} (for [[keyword-uid skel] keyword-uid-skeletons]
                  [keyword-uid
                   (into [] (for [i (range hist-size)]
                              (if (= i (mod old-frame-count hist-size))
                                skel
                                (if (< (count (keyword-uid s-h)) i)
                                  nil
                                  (nth (keyword-uid s-h) i)))))]))))))

(add-watch frame-count
           :update-skeletons-hist update-skeletons-hist)

(def master-vol 1)

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
                   :left-hand 0.12,
                   :right-hand 0.03})

(defn get-inst-vols [ct]
  (let [itv (ct inst-toggles)]
    (let [s (apply + (into [] (for [[k v] inst-vol-map] (* (k itv) v))))]
      (into {} (for [[k v] inst-vol-map] [k (/ (* (k itv) v) s)])))))

(def joint-y-stage-range {:head [-1000 500],
                    :left-hand [-500 500],
                    :right-hand [-500 500],
                    :left-foot [-1000 500],
                    :right-foot [-1000 500]})

(def velocity-frac 0.01)

(def joint-freq-range {:head [60 250],
                       :left-foot [120 240],
                       :right-foot [30 120]})

(def joint-chords {:left-hand (overtone/chord :C3 #{0 4 7 12}),
                   :right-hand (overtone/chord :F3 :major7)})

;; for changes during performance
;; one of [:feet, :hands, :all]
(def ctl-type (atom :feet))

;; reset the insts whilst hacking on code
(defn reset-insts []
  (overtone/stop)
  (reset! keyword-uid-insts {}))

(defn ctl-sound
  [the-key the-ref
   old-skeleton-hists new-skeleton-hists]
  (let [keyword-uids (keys new-skeleton-hists)
        set-keyword-uids (set keyword-uids)
        num-uids (count keyword-uids)]
    (doseq [[keyword-uid insts] @keyword-uid-insts]
      (if (contains? set-keyword-uids keyword-uid)
        (let [skel-hist (keyword-uid new-skeleton-hists)]
          (if (ready skel-hist)
            (doseq [joint ctl-joints]
              (let [inst-id (joint insts)
                    joint-vol (joint (get-inst-vols @ctl-type))]
                (if (= 0 joint-vol)
                  (overtone/ctl inst-id :vol 0)
                  (let [y-stage-range (joint joint-y-stage-range)
                        min-y (first y-stage-range)
                        max-y (second y-stage-range)
                        avg-y (/ (:sum (:y (joint (skeleton-range skel-hist)))) hist-size)
                        freq
                        (if (contains? (set hands) joint)
                          (let [chord (joint joint-chords)]
                            (overtone/midi->hz
                             (overtone/note
                              (nth chord
                                   (int (overtone/scale-range
                                         avg-y
                                         min-y
                                         max-y
                                         0
                                         (- (count chord) 1)))))))
                          (let [freq-range (joint joint-freq-range)]
                            (overtone/scale-range
                             avg-y
                             min-y
                             max-y
                             (first freq-range)
                             (second freq-range))))]
                    (overtone/ctl
                     inst-id
                     :freq
                     freq
                     :vol
                     (overtone/scale-range
                      (math/abs
                       (:max
                           (:y (joint
                                (skeleton-range (velocity skel-hist))))))
                      0 (* velocity-frac (:x stage-size) hist-size)
                      0 (/ (* master-vol joint-vol) num-uids)))))))))
        (doseq [[keyword-uid inst] insts] (overtone/ctl inst :vol 0)) ))))

(add-watch skeletons-hist
           :skeletons-hist-watcher ctl-sound)

;; for hacking
;; (reset-insts)

;; during performance
;; (reset! ctl-type :hands)
;; (reset! ctl-type :all)
;; (reset! ctl-type :feet)
