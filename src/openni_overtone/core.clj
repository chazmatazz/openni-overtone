(ns openni-overtone.core
  (:require [bifocals.core :as bifocals]
            [quil.core :as quil]
            [overtone.live :as overtone]
            [clojure.math.numeric-tower :as math]))

;; An example of drawing all available skeletons, in 2D. Shows how to project
;; skeletons, which are in 3D, down to 2D. The 2D coordinates are the pixel
;; coordinates of the kinect's depth image.
;; hacks in some overtone

(defn setup []
  (quil/smooth)
  ; This function connects to the kinect. You must call it first. Otherwise,
  ; there will be NPEs.
  (bifocals/setup)
  (quil/frame-rate 30))

(defn draw-line
  [p1 p2]
  (quil/line (:x p1) (:y p1) (:x p2) (:y p2)))

(defn draw-skeleton
  [skeleton]
  (let [; This is an example of destructuring a skeleton, which is just a map
        ; with all the listed keys. The values are also maps, representing
        ; vectors, with :x, :y, and :z keys.
        {:keys [head neck left-shoulder right-shoulder left-elbow right-elbow
                left-hand right-hand torso left-hip right-hip left-knee
                right-knee left-foot right-foot]}
          ; Can also use bifocals/project to project arbitrary 3D vectors down
          ; to 2D coordinates.
          (bifocals/project-skeleton skeleton)]
    (quil/stroke 30 120 180)
    (quil/stroke-weight 3)
    (draw-line head neck)
    (draw-line neck left-shoulder), (draw-line neck right-shoulder)
    (draw-line left-shoulder left-elbow), (draw-line right-shoulder right-elbow)
    (draw-line left-elbow left-hand), (draw-line right-elbow right-hand)
    (draw-line left-shoulder torso), (draw-line right-shoulder torso)
    (draw-line torso left-hip), (draw-line torso right-hip)
    (draw-line left-hip left-knee), (draw-line right-hip right-knee)
    (draw-line left-knee left-foot), (draw-line right-knee right-foot)))

(defn draw []
  (quil/background 0)
  ; You must call this in the draw function. Otherwise, your depth image will be
  ; all black, and user/skeleton tracking will not work.
  (bifocals/tick)
  (quil/image (bifocals/depth-image) 0 0)
  (quil/no-stroke)
  (quil/fill 0 255 0 128)
  ; The uids of the on-screen users are stored in a set at the
  ; `bifocals/users` atom. Note that users are added to this set as soon as they
  ; are recognized, which is before they have been calibrated, so their skeleton
  ; skeleton data may not be available. You can check if any users are in that
  ; calibrating state with `(some @bifocals/users (keys @bifocals/skeletons))`.
  (doseq [uid @bifocals/users]
    (quil/rect 10 (- (* 30 uid) 20) 20 20))
  ; The currently tracked skeletons are stored as values of a map in the atom
  ; `bifocals/skeletons`. The map is keyed by the uids of the users that the
  ; skeletons correspond to.
  (doseq [skeleton (vals @bifocals/skeletons)]
    (draw-skeleton skeleton)))

(quil/defsketch kinect
  :title "Unfolding Perception"
  :setup setup
  :draw draw
  ; If your depth image seems truncated, or the window is much larger than it,
  ; check its dimensions by calling `bifocals/depth-width` and
  ; `bifocals/depth-height`, and then adjust the sketch size accordingly.
  :size [640 480])

;; Sound code

(def hist-size 10)

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

(defn deriv [new-v old-v]
  (into {} (for [joint (keys bifocals.skeleton/joints)]
             [joint (into {} (for [axis [:x :y :z]]
                        [axis (-
                               (axis (joint new-v))
                               (axis (joint old-v)))]))])))

(defn deriv1 [skeleton-hist n]
  (deriv (nth skeleton-hist n) (nth skeleton-hist (+ n 1))))

(defn deriv2 [skeleton-hist n]
  (deriv (deriv1 skeleton-hist n) (deriv1 skeleton-hist (+ n 1))))

(defn velocity [skeleton-hist]
  (deriv1 skeleton-hist 0))

(defn acceleration [skeleton-hist]
  (deriv2 skeleton-hist 0))

(defn ready [skeleton-hist]
  (and
   (>= (count skeleton-hist) hist-size)
   (not (reduce (fn [acc v] (or acc v)) false (map nil? skeleton-hist)))))

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
                    (/ (* (:downstage-right quadrant-vols) quadrant-vol quadrant-scale-vol)
                       num-uids)
                    :upstage-left-vol
                    (/ (* (:upstage-left quadrant-vols) quadrant-vol quadrant-scale-vol)
                       num-uids)
                    :downstage-left-vol
                    (/ (* (:downstage-left quadrant-vols) quadrant-vol quadrant-scale-vol)
                       num-uids)
                    :head-freq
                    (* 0.7 (- 1000 (:y (:head (first skeleton-hist)))))
                    :head-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (:head (velocity skeleton-hist))))
                        0 (* 0.01 (stage-size :x))
                        0 0.2)
                       num-uids)
                    :left-foot-freq
                    (* 0.7 (+ 440 (:y (:left-foot (first skeleton-hist)))))
                    :left-foot-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (:left-foot (velocity skeleton-hist))))
                        0 (* 0.01 (stage-size :x))
                        0 0.2) num-uids)
                    :left-hand-freq
                    (* 0.7 (+ 440 (:y (:left-hand (first skeleton-hist)))))
                    :left-hand-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (:left-hand (velocity skeleton-hist))))
                        0 (* 0.01 (stage-size :x))
                        0 0.4) num-uids)))))

(def buf-b-size 36)
(def buf-b (overtone/buffer buf-b-size))
(overtone/buffer-write! buf-b 0 (map #(+ 12 %) [50 50 54 50 57 50 45 49 50 50 54 50 57 50 45 49 50 50 54 50 57 50 45 49 50 50 54 50 57 50 45 49 50 50 54 50 57 50 45 49 50 50 50 50]))

(overtone/definst inst-b [rate 20 left-vol 0 right-vol 0]
      (let [trig (overtone/impulse:kr rate)
            indexes (overtone/dseq (range 8) overtone/INF)
            freqs (overtone/dbufrd buf-b indexes)
            note-gen (overtone/demand:kr trig 0 freqs)
            src (overtone/sin-osc (overtone/midicps note-gen))]
        (* [left-vol right-vol] src)))

(defn ctl-b [inst-id skeleton-hist num-uids]
  (when (ready skeleton-hist)
    (overtone/buffer-set! buf-b
                          (int (overtone/scale-range
                                  (:x (:head (first skeleton-hist)))
                                  (:x (:min stage)) (:x (:max stage))
                                  0 (- buf-b-size 1)))
                 (math/floor (overtone/scale-range
                         (:x (:right-foot (first skeleton-hist)))
                         (:x (:min stage)) (:x (:max stage))
                         40 80)))
    (overtone/ctl inst-id
                  :rate 8
                  :left-vol (* 0 (overtone/scale-range
                             (:y (:left-hand (first skeleton-hist)))
                             (:y (:min stage)) (:y (:max stage))
                             0 (/ 1 num-uids)))
                  :right-vol (* 0 (overtone/scale-range
                              (:z (:right-hand (first skeleton-hist)))
                              (:z (:min stage)) (:z (:max stage))
                              0 (/ 1 num-uids))))))

;; skeletons vector
;; each entry in the vector is a map from uid to skeleton
(def keyword-skeletons-hist (atom []))
;; map from keyword-uid to instrument
(def uid-inst (atom {}))

;; helper function
;; given a skeletons-hist vector, extract the keys
;; from each item in the vector, e.g. the keyword-uids
(defn skeleton-keys [s-hist]
  (keys (apply merge s-hist)))

(defn keywordize [skeletons]
  (into {} (for [[k v] (seq skeletons)] [(keyword (str k)) v])))

;; convert a map indexed by index
;; to a map indexed by keyword-uid
(defn skeleton-pivot [keyword-uids keyword-skels-hist]
    (into {}
          (for [keyword-uid keyword-uids]
            [keyword-uid
             (into [] (for [keyword-skels keyword-skels-hist]
                        (get keyword-skels keyword-uid nil)))])))

(defn on-skeletons-change [the-key the-ref old-skeletons new-skeletons]
  (let [old-keyword-skeletons-hist @keyword-skeletons-hist
        new-keyword-skeletons-hist
        (cons (keywordize new-skeletons)
              (take (- hist-size 1) old-keyword-skeletons-hist))
        old-keyword-uids (skeleton-keys old-keyword-skeletons-hist)
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
            (let [inst-id (if (odd? num-new-keyword-uids) "a" "b")]
              (swap! uid-inst assoc keyword-uid
                      [inst-id (if (= "a" inst-id) (inst-a) (inst-a))])))
          (let [skeleton-hist (keyword-uid new-keyword-uid-skeleton-hists)
                v (keyword-uid @uid-inst)]
            (if (= "a" (first v))
              (ctl-a (second v) skeleton-hist num-new-keyword-uids)
              (ctl-a (second v) skeleton-hist num-new-keyword-uids))))
        (let [v (keyword-uid @uid-inst)]
          (when-not (nil? v)
            (overtone/kill (second v))
            (swap! uid-inst dissoc keyword-uid)))))
    (reset! keyword-skeletons-hist new-keyword-skeletons-hist)))

(add-watch bifocals/skeletons :skeletons-watcher on-skeletons-change)0
