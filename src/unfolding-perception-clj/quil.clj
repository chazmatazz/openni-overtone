(ns unfolding-perception
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

(overtone/definst head-sound [freq 440 vol 0] (* vol (overtone/sin-osc freq)))

(overtone/definst br-sound [vol 0] (* vol (overtone/lf-saw 30)))

(overtone/definst fr-sound [vol 0] (* vol (overtone/sin-osc 440)))

(overtone/definst bl-sound
  [vol 0]
  (let [base-freq 60
        part-vol (/ vol 3.0)]
    (* part-vol (overtone/lf-tri base-freq))
    (* part-vol (overtone/lf-tri (* base-freq (/ 2.0 3))))
    (* part-vol (overtone/lf-tri (* base-freq 2)))))

(overtone/definst fl-sound [vol 0] (* vol (overtone/sin-osc 220)))

(defn start-sound []
  (head-sound)
  (br-sound)
  (fr-sound)
  (bl-sound)
  (fl-sound))

(start-sound)

(defn control-sound
  [uid old-skeleton new-skeleton]
  (let [kinectspace
        {:min {:x -400, :y -400, :z 0},
         :max {:x 400, :y 400, :z 5000},
         :center {:x 0, :y 0, :z 1500}}
        quadrant
        (+
         (if (> (:x (:neck new-skeleton)) (:x (:center kinectspace))) 0 1)
         (* 2 (if (> (:z (:neck new-skeleton)) (:z (:center kinectspace))) 0 1)))]
    (let [head-freq-range (if (= quadrant 2) [100 30] [220 60])
          head-vol-max
          (condp = quadrant
            0 0.6
            1 0.8
            2 0.5
            3 0.8)]
      (let
          [
           head-freq
           (overtone/scale-range
            (:x (:head new-skeleton))
            (:x (:min kinectspace)) (:x (:max kinectspace))
            (first head-freq-range) (second head-freq-range))
           head-vol
           (overtone/scale-range
            (math/abs (- (:y (:head new-skeleton)) (:y (:head old-skeleton))))
            0 50
            0 head-vol-max)
           ]
        (let [r (- 1.0 head-vol-max)]
          (let [vol-vec (condp = quadrant
                          0 [r 0 0 0]
                          1 [0 r 0 0]
                          2 [0 0 r 0]
                          3 [0 0 0 r])]
            (overtone/ctl head-sound :freq head-freq :vol head-vol)
            (overtone/ctl br-sound :vol (first vol-vec))
            (overtone/ctl fr-sound :vol (second vol-vec))
            (overtone/ctl bl-sound :vol (nth vol-vec 2))
            (overtone/ctl fl-sound :vol (nth vol-vec 3))))))))

(defn on-skeletons-change [the-key the-ref old-skeletons new-skeletons]
  (doseq
      [[uid skeleton-hist]
       (merge-with
        (fn [val-in-result val-in-latter] {:old val-in-result, :new val-in-latter}) old-skeletons new-skeletons)]
    (if (contains? skeleton-hist :old)
      (control-sound uid (:old skeleton-hist) (:new skeleton-hist)))))

(add-watch bifocals/skeletons :skeletons-watcher on-skeletons-change)
