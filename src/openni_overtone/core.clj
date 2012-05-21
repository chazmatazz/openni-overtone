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

;; helper function: get the quadrant
(defn quadrant [c]
  (condp = (+ (if (downstage? c) 0 1) (* 2 (if (stage-left? c) 0 1)))
    0 :upstage-right
    1 :downstage-right
    2 :upstage-left
    3 :downstage-left))

;; ------------------------
;; skeleton history helpers
;; ------------------------

(defn most-recent [skeleton-hist joint]
  (joint (first skeleton-hist)))

(defn velocity [skeleton-hist joint]
  (into {} (for [axis [:x :y :z]]
             [axis (-
                    (axis (joint (first skeleton-hist)))
                    (axis (joint (second skeleton-hist))))])))

(defn ready [skeleton-hist]
  ; (println (first skeleton-hist))
  ; (println (second skeleton-hist))
  (and (not= (first skeleton-hist) nil) (not= (second skeleton-hist) nil)))

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

(overtone/definst inst-b
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

(defn ctl-a [skeleton-hist num-uids]
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
              (quadrant (most-recent skeleton-hist :neck)) 1))]
      (overtone/ctl inst-a
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
                    (* 0.7 (- 1000 (:y (most-recent skeleton-hist :head))))
                    :head-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (velocity skeleton-hist :head)))
                        0 (* 0.01 (stage-size :x))
                        0 0.2)
                       num-uids)
                    :left-foot-freq
                    (* 0.7 (+ 440 (:y (most-recent skeleton-hist :left-foot))))
                    :left-foot-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (velocity skeleton-hist :left-foot)))
                        0 (* 0.01 (stage-size :x))
                        0 0.2) num-uids)
                    :left-hand-freq
                    (* 0.7 (+ 440 (:y (most-recent skeleton-hist :left-hand))))
                    :left-hand-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (velocity skeleton-hist :left-hand)))
                        0 (* 0.01 (stage-size :x))
                        0 0.4) num-uids)))))

(defn ctl-b [skeleton-hist num-uids]
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
              (quadrant (most-recent skeleton-hist :neck)) 1))]
      (overtone/ctl inst-b
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
                    (* 0.7 (- 1000 (:y (most-recent skeleton-hist :head))))
                    :head-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (velocity skeleton-hist :head)))
                        0 (* 0.01 (stage-size :x))
                        0 0.2)
                       num-uids)
                    :left-foot-freq
                    (* 0.7 (+ 440 (:y (most-recent skeleton-hist :left-foot))))
                    :left-foot-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (velocity skeleton-hist :left-foot)))
                        0 (* 0.01 (stage-size :x))
                        0 0.2) num-uids)
                    :left-hand-freq
                    (* 0.7 (+ 440 (:y (most-recent skeleton-hist :left-hand))))
                    :left-hand-vol
                    (/ (overtone/scale-range
                        (math/abs (:x (velocity skeleton-hist :left-hand)))
                        0 (* 0.01 (stage-size :x))
                        0 0.4) num-uids)))))

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
          (cons (keywordize new-skeletons) (take 10 old-keyword-skeletons-hist))
          old-keyword-uids (skeleton-keys old-keyword-skeletons-hist)
          new-keyword-uids (skeleton-keys new-keyword-skeletons-hist)
          old-keyword-uid-skeleton-hists
          (skeleton-pivot old-keyword-uids old-keyword-skeletons-hist)
          new-keyword-uid-skeleton-hists
          (skeleton-pivot new-keyword-uids new-keyword-skeletons-hist)
          keyword-uids (set (concat old-keyword-uids new-keyword-uids))
          num-new-keyword-uids (count new-keyword-uids)]
      (doseq [keyword-uid keyword-uids]
        (if (contains? @uid-inst keyword-uid)
          (do
            (if (some #{keyword-uid} keyword-uids)
              (let [skeleton-hist (keyword-uid new-keyword-uid-skeleton-hists)]
                (if (= "a" (keyword-uid @uid-inst))
                  (ctl-a skeleton-hist num-new-keyword-uids)
                  (ctl-b skeleton-hist num-new-keyword-uids)))
              (do
                (if (= "a" (keyword-uid @uid-inst))
                  (overtone/kill inst-a)
                  (overtone/kill inst-b))
                (reset! uid-inst (dissoc @uid-inst keyword-uid)))))
          (let [inst-id (if (odd? num-new-keyword-uids) "a" "b")]
            (if (= "a" inst-id)
              (inst-a)
              (inst-b))
            (reset! uid-inst (assoc @uid-inst keyword-uid inst-id)))))
      (reset! keyword-skeletons-hist new-keyword-skeletons-hist)))

(add-watch bifocals/skeletons :skeletons-watcher on-skeletons-change)
