(ns openni-overtone.core
  (:require [bifocals.core :as bifocals]
            [quil.core :as quil]
            [overtone.live :as overtone]
            [clojure.math.numeric-tower :as math]))

(def joint-pairs
  [[:head :neck] [:neck :left-shoulder] [:neck :right-shoulder]
   [:left-shoulder :left-elbow] [:right-shoulder :right-elbow]
   [:left-elbow :left-hand] [:right-elbow :right-hand]
   [:left-shoulder :torso] [:right-shoulder :torso]
   [:torso :left-hip] [:torso :right-hip]
   [:left-hip :left-knee] [:right-hip :right-knee]
   [:left-knee :left-foot] [:right-knee :right-foot]])


(def hist-size 2)

;; skeletons vector
;; each entry in the vector is a map from uid to skeleton
(def keyword-skeletons-hist (atom []))

(defn ready [skeleton-hist]
  (and
   (>= (count skeleton-hist) hist-size)
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

;; map from keyword-uid to instrument
(def uid-inst (atom {}))

;; Sound code

;; ---------
;; quadrants
;; ---------

;; the definition of the stage
(def stage
  {:min {:x -400, :y -800, :z 0},
   :max {:x 400, :y 2000, :z 5000},
   :center {:x 0, :y 0, :z 2500}})

;; the size of an axis of the stage
(defn stage-size [axis]
  (- (axis (:max stage)) (axis (:min stage))))

;; which halfspace is coordinate in for axis?
(defn halfspace? [c axis] (> (axis c) (axis (:center stage))))

;; is coordinate stage left?
(defn stage-left? [c] (halfspace? c :x))

;; is coordinate up/down?
(defn down? [c] (halfspace? c :y))

;; is coordinate downstage?
(defn downstage? [c] (halfspace? c :z))

;; helper function: get the quadrant num
(defn quadrant-num [c]
  (+ (if (downstage? c) 0 1) (* 2 (if (stage-left? c) 0 1))))

;; helper function: get the quadrant keyword
(defn quadrant [c]
    (condp = (quadrant-num c)
    0 :upstage-right
    1 :downstage-right
    2 :upstage-left
    3 :downstage-left))

;; ------------------------
;; skeleton history helpers
;; ------------------------

;; derivatives

;; given 2 skeletons, calculate the derivative
(defn skeleton-derive [new-skel old-skel]
  (into {} (for [joint (keys bifocals.skeleton/joints)]
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
        (for [joint (keys bifocals.skeleton/joints)]
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
                      overtone/INF skeleton-hist)}]))])))

;; ----------------------
;; instrument definitions
;; ----------------------

(overtone/definst inst-a
  [upstage-right-vol 0
   downstage-right-vol 0
   upstage-left-vol 0
   downstage-left-vol 0
   head-freq 440
   head-vol 0
   left-foot-freq 440
   left-foot-vol 0
   left-hand-freq 440
   left-hand-vol 0]
  (+ (* upstage-right-vol (overtone/lf-saw 30))
     (* downstage-right-vol (overtone/sin-osc 440))
     (let [base-freq 60]
       (* upstage-left-vol
          (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)])))
     (* downstage-left-vol (overtone/sin-osc 220))
     (* head-vol (overtone/sin-osc head-freq))
     (* left-foot-vol (overtone/sin-osc left-foot-freq))
     (* left-hand-vol (overtone/square left-hand-freq))))

(defn ctl-a [inst-id skeleton-hist num-uids]
  (when (ready skeleton-hist)
    (let [quadrant-vol 0.2
          quadrant-scale-vol 0.7
          quadrant-vols
          (let [vs
                {:upstage-right 0,
                 :downstage-right 0,
                 :upstage-left 0,
                 :downstage-left 0}
                ]
            (assoc vs
              (quadrant (:neck (first skeleton-hist))) 1))]
      (overtone/ctl inst-id
                    :upstage-right-vol
                    (/ (* (:upstage-right quadrant-vols) quadrant-vol) num-uids)
                    :downstage-right-vol
                    (/ (* (:downstage-right quadrant-vols)
                          quadrant-vol quadrant-scale-vol)
                       num-uids)
                    :upstage-left-vol
                    (/ (* (:upstage-left quadrant-vols)
                          quadrant-vol quadrant-scale-vol)
                       num-uids)
                    :downstage-left-vol
                    (/ (* (:downstage-left quadrant-vols)
                          quadrant-vol quadrant-scale-vol)
                       num-uids)
                    :head-freq
                    (* 0.7 (- 1000 (:y (:head (first skeleton-hist)))))
                    :head-vol
                    (overtone/scale-range
                     (math/abs (:x (:head (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :left-foot-freq
                    (* 0.7 (+ 440 (:y (:left-foot (first skeleton-hist)))))
                    :left-foot-vol
                    (overtone/scale-range
                     (math/abs (:x (:left-foot
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :left-hand-freq
                    (* 0.7 (+ 440 (:y (:left-hand (first skeleton-hist)))))
                    :left-hand-vol
                    (overtone/scale-range
                     (math/abs (:x (:left-hand
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.4 num-uids))))))


(def buf-b-size 8)
(def buf-b (overtone/buffer buf-b-size))
(overtone/buffer-write! buf-b 0 (map #(+ 12 %) [50 50 54 50 57 50 45 49]))

(overtone/definst inst-b [rate 20 left-vol 0 right-vol 0]
      (let [trig (overtone/impulse:kr rate)
            indexes (overtone/dseq (range buf-b-size) overtone/INF)
            freqs (overtone/dbufrd buf-b indexes)
            note-gen (overtone/demand:kr trig 0 freqs)
            src (overtone/sin-osc (overtone/midicps note-gen))]
        (* [left-vol right-vol] src)))

(defn ctl-b [inst-id skeleton-hist num-uids]
  (when (ready skeleton-hist)
    (overtone/ctl inst-id
                  :rate (* 0 (overtone/scale-range
                             (:y (:head (first (velocity skeleton-hist))))
                             (:y (:min stage)) (:y (:max stage))
                             0 (/ 1.0 num-uids)))
                  :left-vol (* 0 (overtone/scale-range
                             (:y (:left-hand (first skeleton-hist)))
                             (:y (:min stage)) (:y (:max stage))
                             0 (/ 1.0 num-uids)))
                  :right-vol (* 0 (overtone/scale-range
                              (:z (:right-hand (first skeleton-hist)))
                              (:z (:min stage)) (:z (:max stage))
                              0 (/ 1.0 num-uids))))))

(overtone/definst inst-c
  [head-freq 440
   head-vol 0
   right-hand-freq 440
   right-hand-vol 0
   left-hand-freq 440
   left-hand-vol 0]
  (+ (* head-vol (overtone/sin-osc head-freq))
     (* right-hand-vol (overtone/sin-osc right-hand-freq))
     (* left-hand-vol (overtone/square left-hand-freq))))

(defn ctl-c [inst-id skeleton-hist num-uids]
  (when (ready skeleton-hist)
    (overtone/ctl inst-id
                    :head-freq
                    (* 0.7 (- 1000 (:y (:head (first skeleton-hist)))))
                    :head-vol
                    (overtone/scale-range
                     (math/abs (:x (:head (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :right-hand-freq
                    (* 0.7 (+ 440 (:y (:right-hand (first skeleton-hist)))))
                    :right-hand-vol
                    (overtone/scale-range
                     (math/abs (:x (:right-hand
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :left-hand-freq
                    (* 0.7 (+ 440 (:y (:left-hand (first skeleton-hist)))))
                    :left-hand-vol
                    (overtone/scale-range
                     (math/abs (:x (:left-hand
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.4 num-uids)))))

(overtone/definst inst-d
  [head-freq 440
   head-vol 0
   right-foot-freq 440
   right-foot-vol 0
   left-foot-freq 440
   left-foot-vol 0
   right-hand-freq 440
   right-hand-vol 0
   left-hand-freq 440
   left-hand-vol 0]
  (+ (* head-vol (overtone/sin-osc head-freq))
     (* right-foot-vol (overtone/sin-osc right-foot-freq))
     (* left-foot-vol (overtone/sin-osc left-foot-freq))
     (* right-hand-vol (overtone/square right-hand-freq))
     (* left-hand-vol (overtone/square left-hand-freq))))

(defn ctl-d [inst-id skeleton-hist num-uids]
  (when (ready skeleton-hist)
      (overtone/ctl inst-id
                    :head-freq
                    (* 0.7 (- 1000 (:y (:head (first skeleton-hist)))))
                    :head-vol
                    (overtone/scale-range
                     (math/abs (:x (:head (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :right-foot-freq
                    (* 0.7 (+ 1500 (:y (:right-foot (first skeleton-hist)))))
                    :right-foot-vol
                    (overtone/scale-range
                     (math/abs (:x (:right-foot
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.02 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :left-foot-freq
                    (* 0.7 (+ 1500 (:y (:left-foot (first skeleton-hist)))))
                    :left-foot-vol
                    (overtone/scale-range
                     (math/abs (:x (:left-foot
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.02 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :right-hand-freq
                    (* 0.7 (+ 440 (:y (:right-hand (first skeleton-hist)))))
                    :right-hand-vol
                    (overtone/scale-range
                     (math/abs (:x (:right-hand
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids))
                    :left-hand-freq
                    (* 0.7 (+ 440 (:y (:left-hand (first skeleton-hist)))))
                    :left-hand-vol
                    (overtone/scale-range
                     (math/abs (:x (:left-hand
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.01 (stage-size :x))
                     0 (/ 0.2 num-uids)))))

(overtone/definst inst-e
  [right-foot-freq 440
   right-foot-vol 0
   left-foot-freq 440
   left-foot-vol 0]
  (+ (* right-foot-vol (overtone/sin-osc right-foot-freq))
     (* left-foot-vol (overtone/sin-osc left-foot-freq))))

(defn ctl-e [inst-id skeleton-hist num-uids]
  (when (ready skeleton-hist)
      (overtone/ctl inst-id
                    :right-foot-freq
                    (overtone/scale-range
                     (math/abs (- (:y (:right-foot (first skeleton-hist)))
                                      (:y (:torso (first skeleton-hist)))))
                     0 100
                     440 880)
                    :right-foot-vol
                    (overtone/scale-range
                     (math/abs (:x (:right-foot
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.02 (stage-size :x))
                     0 (/ 0.5 num-uids))
                    :left-foot-freq
                    (* 0.7 (+ 1000 (- (:y (:left-foot (first skeleton-hist)))
                                      (:y (:torso (first skeleton-hist))))))
                    :left-foot-vol
                    (overtone/scale-range
                     (math/abs (:x (:left-foot
                                    (first (velocity skeleton-hist)))))
                     0 (* 0.02 (stage-size :x))
                     0 (/ 0.5 num-uids)))))

(def inst-map
  {:a {:inst inst-a, :color {:r 20, :g 120, :b 180}},
   :b {:inst inst-b, :color {:r 255, :g 0, :b 0}},
   :c {:inst inst-c, :color {:r 0, :g 255, :b 0}},
   :d {:inst inst-d, :color {:r 0, :g 255, :b 255}},
   :e {:inst inst-e, :color {:r 255, :g 0, :b 255}}})

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

  (let [skeletons-uid-hist
        (skeleton-pivot
         (skeleton-keys @keyword-skeletons-hist) @keyword-skeletons-hist)]
    (doseq [[keyword-uid skel-hist] skeletons-uid-hist]
      (when (ready skel-hist)
        (quil/stroke-weight 3)
        (dotimes [i 1]
          (let [project-skeleton
                (bifocals/project-skeleton (nth skel-hist i))]
            (if (contains? @uid-inst keyword-uid)
              (let [color (:color ((first (keyword-uid @uid-inst)) inst-map))]
                (quil/stroke (:r color) (:g color) (:b color)
                             (* (/ 255 hist-size) (- hist-size i)))))
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

;; when bifocals changes
(defn on-bifocals-skeletons-change [the-key the-ref old-skeletons new-skeletons]
  (reset! keyword-skeletons-hist
          (cons (keywordize new-skeletons)
                (take (- hist-size 1) @keyword-skeletons-hist))))

(add-watch bifocals/skeletons
           :bifocals-skeletons-watcher on-bifocals-skeletons-change)

;; update uid-inst and control sound
(defn on-keyword-skeletons-hist-change
  [the-key the-ref
   old-keyword-skeletons-hist new-keyword-skeletons-hist]
  (let [old-keyword-uids (skeleton-keys old-keyword-skeletons-hist)
        new-keyword-uids (skeleton-keys new-keyword-skeletons-hist)
        old-keyword-uid-skeleton-hists
        (skeleton-pivot old-keyword-uids old-keyword-skeletons-hist)
        new-keyword-uid-skeleton-hists
        (skeleton-pivot new-keyword-uids new-keyword-skeletons-hist)
        keyword-uids (set (concat old-keyword-uids new-keyword-uids))
        num-new-keyword-uids (count new-keyword-uids)]
    (doseq [keyword-uid keyword-uids]
      (if (some #{keyword-uid} new-keyword-uids)
        (do
          (when-not (contains? @uid-inst keyword-uid)
            (let [inst-keyword (if (odd? num-new-keyword-uids) :d :d)
                  inst-fn (:inst (inst-keyword inst-map))
                  inst-id (inst-fn)]
              (swap! uid-inst assoc keyword-uid
                      [inst-keyword inst-id])))
          (let [skeleton-hist (keyword-uid new-keyword-uid-skeleton-hists)
                v (keyword-uid @uid-inst)
                k (first v)
                inst-id (second v)]
            (if (= :a k)
              (ctl-a inst-id skeleton-hist num-new-keyword-uids)
              (if (= :b k)
                (ctl-b inst-id skeleton-hist num-new-keyword-uids)
                (if (= :c k)
                  (ctl-c inst-id skeleton-hist num-new-keyword-uids)
                  (if (= :d k)
                    (ctl-d inst-id skeleton-hist num-new-keyword-uids)
                    (ctl-e inst-id skeleton-hist num-new-keyword-uids)))))))
        (let [v (keyword-uid @uid-inst)]
          (when-not (nil? v)
            (overtone/kill (second v))
            (swap! uid-inst dissoc keyword-uid)))))))

(add-watch keyword-skeletons-hist
           :keyword-skeletons-hist-watcher on-keyword-skeletons-hist-change)

(defn reset []
  (overtone/stop)
  (reset! uid-inst {}))

(reset)

@uid-inst
