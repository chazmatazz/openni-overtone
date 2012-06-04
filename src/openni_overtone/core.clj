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

;; skeletons vector
;; each entry in the vector is a map from uid to skeleton
(def keyword-skeletons-hist (atom []))

;; is there a skeleton-hist of at least n
(defn ready [skeleton-hist n]
  (and
   (>= (count skeleton-hist) n)
   (not (reduce (fn [acc v] (or acc v)) false (map nil? skeleton-hist)))))

;; convert uids to keywords
(defn keywordize [skeletons]
  (into {} (for [[k v] (seq skeletons)] [(keyword (str k)) v])))

;; helper function
;; given a skeletons-hist vector, extract the keys
;; from each item in the vector, e.g. the keyword-uids
(defn skeleton-keys [s-hist]
  (keys (apply merge s-hist)))

;; convert a map indexed by index
;; to a map indexed by keyword-uid
(defn skeleton-pivot [keyword-uids keyword-skels-hist]
    (into {}
          (for [keyword-uid keyword-uids]
            [keyword-uid
             (into [] (for [keyword-skels keyword-skels-hist]
                        (get keyword-skels keyword-uid nil)))])))

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

(defn draw []
  (quil/background 0)
  ;; You must call this in the draw function. Otherwise, your depth image will be
  ;; all black, and user/skeleton tracking will not work.
  (bifocals/tick)
  (quil/image (bifocals/depth-image) 0 0)
  (quil/stroke-weight 3)
  (quil/stroke 255 0 180)
  (let [ksh (take 1 @keyword-skeletons-hist)
        skeletons-uid-hist
        (skeleton-pivot
         (skeleton-keys ksh) ksh)]
    (doseq [[keyword-uid skel-hist] skeletons-uid-hist]
      (when (ready skel-hist 1)
        (doseq [skel skel-hist]
          (let [project-skeleton
                (bifocals/project-skeleton skel)]
            (doseq [joint-pair joint-pairs]
              (draw-line
               ((first joint-pair) project-skeleton)
               ((second joint-pair) project-skeleton)))))))))

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

;; the ratios aren't working with definst in Overtone 0.6.0

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

;; when bifocals changes
(defn update-keyword-skeletons-hist [the-key the-ref old-skeletons new-skeletons]
  (let [keyword-uid-skeletons (keywordize new-skeletons)]
    ;; add new instruments and configure constant params
    (doseq [keyword-uid (keys keyword-uid-skeletons)]
      (when (not (contains? @keyword-uid-insts keyword-uid))
        (let [insts {:head (inst-sin-osc),
                     :left-hand (inst-sin-third),
                     :right-hand (inst-pulse-fifth),
                     :left-foot (inst-sin-fourth),
                     :right-foot (inst-sin-fourth)}]
          (swap! keyword-uid-insts assoc keyword-uid insts))))
    ;; add to skeletons history
    (swap! keyword-skeletons-hist
           (fn [ksh] (cons keyword-uid-skeletons (take (- hist-size 1) ksh))))))

(add-watch bifocals/skeletons
           :update-keyword-skeletons-hist update-keyword-skeletons-hist)

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
   old-keyword-skeletons-hist new-keyword-skeletons-hist]
  (let [keyword-skeletons-hist new-keyword-skeletons-hist
        keyword-uids (skeleton-keys keyword-skeletons-hist)
        set-keyword-uids (set keyword-uids)
        num-uids (count keyword-uids)
        keyword-uid-skeleton-hists
        (skeleton-pivot keyword-uids keyword-skeletons-hist)
        max-velocity-x (* velocity-frac (:x stage-size))]
    (doseq [[keyword-uid insts] @keyword-uid-insts]
      (if (contains? set-keyword-uids keyword-uid)
        (let [skel-hist (keyword-uid keyword-uid-skeleton-hists)]
          (if (ready skel-hist hist-size)
            (doseq [joint ctl-joints]
              (let [inst-id (joint insts)
                    inst-vols (get-inst-vols @ctl-type)
                    max-vol (/ (* master-vol (joint inst-vols)) num-uids)]
                (if (= max-vol 0)
                  (overtone/ctl inst-id :vol 0)
                  (let [joint-min-y (first (joint joint-y-stage-range))
                        joint-max-y (second (joint joint-y-stage-range))
                        avg-y (/ (:sum (:y (joint (skeleton-range skel-hist)))) hist-size)
                        abs-avg-vel-x
                        (math/abs
                         (/ (:sum
                             (:x (joint
                                  (skeleton-range (velocity skel-hist))))) hist-size))
                        vol (overtone/scale-range
                             abs-avg-vel-x
                             0 max-velocity-x
                             0 max-vol)]
                  (if (contains? (set hands) joint)
                    (let [chord (joint joint-chords)
                          max-chord-idx (- (count chord) 1)]
                      (overtone/ctl
                       inst-id
                       :freq
                       (overtone/midi->hz
                        (overtone/note
                         (nth chord (int (overtone/scale-range
                                          avg-y
                                          joint-min-y joint-max-y
                                          0 max-chord-idx)))))
                       :vol vol))
                    (overtone/ctl
                     inst-id
                     :freq
                     (overtone/scale-range
                      avg-y
                      joint-min-y
                      joint-max-y
                      (first (joint joint-freq-range))
                      (second (joint joint-freq-range)))
                     :vol vol))))))))
        (doseq
            [[keyword-uid inst] insts]
          (overtone/ctl inst :vol 0))))))

(add-watch keyword-skeletons-hist
           :keyword-skeletons-hist-watcher ctl-sound)

;; for hacking
;; (reset-insts)

;; during performance
;; (reset! ctl-type :hands)
;; (reset! ctl-type :all)
