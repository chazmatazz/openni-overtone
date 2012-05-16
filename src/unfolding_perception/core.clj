(ns unfolding-perception.core
  (:require [bifocals.core :as bifocals] [quil.core :as quil] [overtone.live :as overtone] [clojure.math.numeric-tower :as math]))

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


(overtone/definst head-0 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst head-1 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst head-2 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst head-3 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst head-4 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst head-5 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))

(overtone/definst left-foot-0 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst left-foot-1 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst left-foot-2 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst left-foot-3 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst left-foot-4 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))
(overtone/definst left-foot-5 [freq 440 vol 0] (* vol (overtone/sin-osc freq)))

(overtone/definst left-hand-0 [freq 440 vol 0] (* vol (overtone/square freq)))
(overtone/definst left-hand-1 [freq 440 vol 0] (* vol (overtone/square freq)))
(overtone/definst left-hand-2 [freq 440 vol 0] (* vol (overtone/square freq)))
(overtone/definst left-hand-3 [freq 440 vol 0] (* vol (overtone/square freq)))
(overtone/definst left-hand-4 [freq 440 vol 0] (* vol (overtone/square freq)))
(overtone/definst left-hand-5 [freq 440 vol 0] (* vol (overtone/square freq)))

(overtone/definst br-0 [vol 0] (* vol (overtone/lf-saw 30)))
(overtone/definst br-1 [vol 0] (* vol (overtone/lf-saw 30)))
(overtone/definst br-2 [vol 0] (* vol (overtone/lf-saw 30)))
(overtone/definst br-3 [vol 0] (* vol (overtone/lf-saw 30)))
(overtone/definst br-4 [vol 0] (* vol (overtone/lf-saw 30)))
(overtone/definst br-5 [vol 0] (* vol (overtone/lf-saw 30)))

(overtone/definst fr-0 [vol 0] (* vol (overtone/sin-osc 440)))
(overtone/definst fr-1 [vol 0] (* vol (overtone/sin-osc 440)))
(overtone/definst fr-2 [vol 0] (* vol (overtone/sin-osc 440)))
(overtone/definst fr-3 [vol 0] (* vol (overtone/sin-osc 440)))
(overtone/definst fr-4 [vol 0] (* vol (overtone/sin-osc 440)))
(overtone/definst fr-5 [vol 0] (* vol (overtone/sin-osc 440)))

(overtone/definst bl-0   [vol 0]
  (let [base-freq 60]
    (* vol (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)]))))
(overtone/definst bl-1   [vol 0]
  (let [base-freq 60]
    (* vol (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)]))))
(overtone/definst bl-2   [vol 0]
  (let [base-freq 60]
    (* vol (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)]))))
(overtone/definst bl-3   [vol 0]
  (let [base-freq 60]
    (* vol (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)]))))
(overtone/definst bl-4   [vol 0]
  (let [base-freq 60]
    (* vol (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)]))))
(overtone/definst bl-5   [vol 0]
  (let [base-freq 60]
    (* vol (overtone/lf-tri [base-freq (* base-freq 2) (* base-freq 3)]))))

(overtone/definst fl-0 [vol 0] (* vol (overtone/sin-osc 220)))
(overtone/definst fl-1 [vol 0] (* vol (overtone/sin-osc 220)))
(overtone/definst fl-2 [vol 0] (* vol (overtone/sin-osc 220)))
(overtone/definst fl-3 [vol 0] (* vol (overtone/sin-osc 220)))
(overtone/definst fl-4 [vol 0] (* vol (overtone/sin-osc 220)))
(overtone/definst fl-5 [vol 0] (* vol (overtone/sin-osc 220)))

(defn get-head [uid] (condp = (mod uid 6)
                       0 head-0
                       1 head-1
                       2 head-2
                       3 head-3
                       4 head-4
                       5 head-5))

(defn get-left-foot [uid] (condp = (mod uid 6)
                       0 left-foot-0
                       1 left-foot-1
                       2 left-foot-2
                       3 left-foot-3
                       4 left-foot-4
                       5 left-foot-5))

(defn get-left-hand [uid] (condp = (mod uid 6)
                       0 left-hand-0
                       1 left-hand-1
                       2 left-hand-2
                       3 left-hand-3
                       4 left-hand-4
                       5 left-hand-5))

(defn get-br [uid] (condp = (mod uid 6)
                       0 br-0
                       1 br-1
                       2 br-2
                       3 br-3
                       4 br-4
                       5 br-5))

(defn get-fr [uid] (condp = (mod uid 6)
                       0 fr-0
                       1 fr-1
                       2 fr-2
                       3 fr-3
                       4 fr-4
                       5 fr-5))

(defn get-bl [uid] (condp = (mod uid 6)
                       0 bl-0
                       1 bl-1
                       2 bl-2
                       3 bl-3
                       4 bl-4
                       5 bl-5))

(defn get-fl [uid] (condp = (mod uid 6)
                       0 fl-0
                       1 fl-1
                       2 fl-2
                       3 fl-3
                       4 fl-4
                       5 fl-5))

(defn start-sound []
  (doseq [i (range 6)]
         ((get-head i))
         ((get-left-foot i))
         ((get-left-hand i))
         ((get-br i))
         ((get-fr i))
         ((get-bl i))
         ((get-fl i))))

(start-sound)

(def kinectspace
  {:min {:x -400, :y -800, :z 0},
   :max {:x 400, :y 2000, :z 5000},
   :center {:x 0, :y 0, :z 2500}})

(defn get-quadrant [skeleton]
  (+ (if (> (:z (:neck skeleton)) (:z (:center kinectspace))) 0 1)
     (* 2 (if (> (:x (:neck skeleton)) (:x (:center kinectspace))) 0 1))))

(defn control-sound
  [uid old-skeleton new-skeleton]
  (let
      [old-quadrant (get-quadrant old-skeleton)
       new-quadrant (get-quadrant new-skeleton)
       head-freq-range
       (if (= new-quadrant 2) ;bl
         {:lower 660, :upper 440}
         {:lower 600, :upper 200})
       head-vol-min 0
       head-vol-max
       (condp = new-quadrant
         0 0.2 ; br
         1 0.2 ;fr
         2 0.2 ; bl
         3 0.2) ;fl
       left-foot-vol-min 0
       left-foot-vol-max
       (condp = new-quadrant
         0 0.2 ; br
         1 0.2 ;fr
         2 0.2 ; bl
         3 0.2) ;fl
       left-hand-vol-min 0
       left-hand-vol-max
       (condp = new-quadrant
         0 0.4 ; br
         1 0.4 ;fr
         2 0.4 ; bl
         3 0.4) ;fl
       head-freq
       (* 0.7 (- 1000 (:y (:head new-skeleton))))
       head-vol
       (/ (overtone/scale-range
        (math/abs (- (:x (:head new-skeleton)) (:x (:head old-skeleton))))
        0 (* 0.01 (- (:x (:max kinectspace)) (:x (:min kinectspace))))
        head-vol-min head-vol-max) 6)
       left-foot-freq
       (* 0.7 (+ 440 (:y (:left-foot new-skeleton))))
       left-foot-vol
       (/ (overtone/scale-range
        (math/abs (- (:x (:left-foot new-skeleton)) (:x (:left-foot old-skeleton))))
        0 (* 0.01 (- (:x (:max kinectspace)) (:x (:min kinectspace))))
        left-foot-vol-min left-foot-vol-max) 6)
       left-hand-freq
       (* 0.7 (+ 440 (:y (:left-hand new-skeleton))))
       left-hand-vol
       (/ (overtone/scale-range
        (math/abs (- (:x (:left-hand new-skeleton)) (:x (:left-hand old-skeleton))))
        0 (* 0.01 (- (:x (:max kinectspace)) (:x (:min kinectspace))))
        left-hand-vol-min left-hand-vol-max) 6)
       r (* 0 (/ (- 1.0 head-vol-max) 6))
       r2 (* 0 0.7 r)
       vol-vec (condp = new-quadrant
                 0 [r 0 0 0] ;br
                 1 [0 r2 0 0] ;fr
                 2 [0 0 r2 0] ;bl
                 3 [0 0 0 r2])
       ]
    (overtone/ctl (get-head uid) :freq head-freq :vol head-vol)
    (overtone/ctl (get-left-foot uid) :freq left-foot-freq :vol left-foot-vol)
    (overtone/ctl (get-left-hand uid) :freq left-hand-freq :vol left-hand-vol)
    (overtone/ctl (get-br uid) :vol (first vol-vec))
    (overtone/ctl (get-fr uid) :vol (second vol-vec))
    (overtone/ctl (get-bl uid) :vol (nth vol-vec 2))
    (overtone/ctl (get-fl uid) :vol (nth vol-vec 3))))

(defn on-skeletons-change [the-key the-ref old-skeletons new-skeletons]
  (doseq
      [[uid skeleton-hist]
       (merge-with
        (fn [val-in-result val-in-latter] {:old val-in-result, :new val-in-latter}) old-skeletons new-skeletons)]
    (if (contains? skeleton-hist :old)
      (control-sound uid (:old skeleton-hist) (:new skeleton-hist)))))

(add-watch bifocals/skeletons :skeletons-watcher on-skeletons-change)
