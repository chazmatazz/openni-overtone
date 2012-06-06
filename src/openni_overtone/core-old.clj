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

;; buffer size of keyword-skeletons-hist
;; Instruments should use (take n @keyword-skeletons-hist)
;; for speed in calculating derivatives
(def hist-size 10)

;; save some phrases for playback or analysis
(def phrases (atom []))

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

;; ---------
;; quadrants
;; ---------

;; the definition of the stage
;; this probably needs tweaking
(def stage
  {:min {:x -400, :y -800, :z 0},
   :max {:x 400, :y 2000, :z 5000},
   :center {:x 0, :y 0, :z 2500}})

;; the size of an axis of the stage
(def stage-size
  (into {}
        (for [axis [:x :y :z]]
            [axis (- (axis (:max stage)) (axis (:min stage)))])))

;; which halfspace is coordinate in for axis?
(defn halfspace? [c axis] (> (axis c) (axis (:center stage))))

;; is coordinate stage left?
(defn stage-left? [c] (halfspace? c :x))

;; is coordinate up/down?
(defn down? [c] (halfspace? c :y))

;; is coordinate downstage?
(defn downstage? [c] (halfspace? c :z))

;; helper function: get the quadrant num from a coord
(defn quadrant-num [c]
  (+ (if (downstage? c) 0 1) (* 2 (if (stage-left? c) 0 1))))

;; helper function: get the quadrant keyword from a coord
(defn quadrant [c]
    (condp = (quadrant-num c)
    0 :upstage-right
    1 :downstage-right
    2 :upstage-left
    3 :downstage-left))

;; ------------------------
;; skeleton history helpers
;; ------------------------

;; use these functions on (take n ...) skeleton-hists for speed

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
  (quil/scale 2)
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
               ((second joint-pair) project-skeleton))))))))
  )

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

(overtone/definst inst-triple-tri
  [vol 0
   freq 60]
  (* vol (overtone/lf-tri [freq (* freq 2) (* freq 3)])))

(overtone/definst inst-sin-major-third
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc [freq (* freq (/ 5 4))])))

(overtone/definst inst-sin-fourth
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc [freq (* freq (/ 4 3))])))

(overtone/definst inst-sin-fifth
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc [freq (* freq (/ 4 3))])))

(overtone/definst inst-pulse-fifth
  [freq 440
   vol 0]
  (* vol (overtone/pulse [freq (* freq (/ 3 2))])))

(overtone/definst inst-sin-osc
  [freq 440
   vol 0]
  (* vol (overtone/sin-osc freq)))

(overtone/definst inst-square
  [freq 440
   vol 0]
  (* vol (overtone/square freq)))

(overtone/definst inst-saw
  [freq 440
   vol 0]
  (* vol (overtone/saw freq)))

;; uid-inst
(def keyword-uid-insts (atom {}))

;; when bifocals changes
(defn update-keyword-skeletons-hist [the-key the-ref old-skeletons new-skeletons]
  (let [keyword-uid-skeletons (keywordize new-skeletons)]
    ;; add new instruments and configure constant params
    (doseq [keyword-uid (keys keyword-uid-skeletons)]
      (when (not (contains? @keyword-uid-insts keyword-uid))
        (let [insts {:upstage-right (inst-saw),
                     :downstage-right (inst-sin-osc),
                     :upstage-left (inst-triple-tri),
                     :downstage-left (inst-sin-osc),
                     :head (inst-sin-osc),
                     :left-hand (inst-sin-major-third),
                     :right-hand (inst-sin-major-third),
                     :left-foot (inst-sin-fourth),
                     :right-foot (inst-sin-fifth)}]
          (swap! keyword-uid-insts assoc keyword-uid insts)
          (overtone/ctl (:upstage-right insts) :freq 30)
          (overtone/ctl (:downstage-right insts) :freq 440)
          (overtone/ctl (:upstage-left insts) :freq 60)
          (overtone/ctl (:downstage-right insts) :freq 220))))
    ;; add to skeletons history
    (reset! keyword-skeletons-hist
          (cons keyword-uid-skeletons
                (take (- hist-size 1) @keyword-skeletons-hist)))))

(add-watch bifocals/skeletons
           :update-keyword-skeletons-hist update-keyword-skeletons-hist)

(def master-vol-default 1)

(def ctl-joints [:head :left-hand :right-hand :left-foot :right-foot])

(def ctls (cons :quadrant ctl-joints))

(def osc-inst-toggle-defaults
  (assoc (into {} (for [ctl ctl-joints] [ctl 1])) :quadrant 0))

(def osc-inst-vol-defaults {:quadrant 0.0,
                                :head 0.15,
                                :left-foot 0.3,
                                :right-foot 0.4,
                                :left-hand 0.075,
                                :right-hand 0.075})

(def osc-inst-vol-arm-defaults {:quadrant 0.0,
                            :head 0.0,
                             :left-foot 0.0,
                             :right-foot 0.0,
                             :left-hand 0.5,
                            :right-hand 0.5})

(def osc-inst-vol-leg-defaults {:quadrant 0.0,
                            :head 0.0,
                             :left-foot 0.5,
                             :right-foot 0.5,
                             :left-hand 0.0,
                             :right-hand 0.0})


(def quadrants [:upstage-right :downstage-right :upstage-left :downstage-left])

(def osc-quadrant-toggle-defaults
  (into {} (for [q quadrants] [q 1])))
(def osc-quadrant-vol-defaults
  (assoc (into {} (for [q quadrants] [q 0.7])) :upstage-right 1))

(def master-vol (atom 0))

(def osc-inst-toggles (atom osc-inst-toggle-defaults))

(def osc-inst-vols (atom osc-inst-vol-defaults))

(def osc-quadrant-toggles (atom osc-quadrant-toggle-defaults))

(def osc-quadrant-vols (atom osc-quadrant-vol-defaults))

(defn set-touchosc-defaults [c]
  (overtone/osc-send c "/1/master-vol" master-vol-default)

  (apply
   overtone/osc-send
   (cons c
         (cons "/1/osc-inst-toggles"
               (map (fn [k] (k osc-inst-toggle-defaults)) ctls))))

  (apply
   overtone/osc-send
   (cons c
         (cons "/1/osc-inst-vols"
               (map (fn [k] (k @osc-inst-vol-defaults)) ctls))))

  (apply
   overtone/osc-send
   (cons c (cons "/1/osc-quadrant-toggles"
                 (map (fn [k] (k @osc-quadrant-toggle-defaults)) quadrants))))

  (apply
   overtone/osc-send
   (cons c (cons "/1/osc-quadrant-vols"
                 (map (fn [k] (k @osc-quadrant-vol-defaults)) quadrants)))))

(def hands (set [:left-hand :right-hand]))

(def freq-range {:head [60 250],
                 :left-foot [120 240],
                 :right-foot [30 120]})

(def note-chords {:left-hand [48 52 55 60], ; C E G B
                  :right-hand [53 57 60 64], ; F A C E
                  })

(def quadrant-defaults
  (into {} (for [q quadrants] [q 0])))

(def ctl-counter (atom 0))

(def ctl-mod 10)

(defn ctl-sound
  [the-key the-ref
   old-keyword-skeletons-hist new-keyword-skeletons-hist]
    (let [n 10
          keyword-skeletons-hist (take n new-keyword-skeletons-hist)
          keyword-uids (skeleton-keys keyword-skeletons-hist)
          set-keyword-uids (set keyword-uids)
          num-uids (count keyword-uids)
          keyword-uid-skeleton-hists
          (skeleton-pivot keyword-uids keyword-skeletons-hist)]
      (doseq [[keyword-uid insts] @keyword-uid-insts]
        (if (contains? set-keyword-uids keyword-uid)
          (let [skel-hist (keyword-uid keyword-uid-skeleton-hists)]
            (if (ready skel-hist 1)
              (let [quadrant-state
                    (assoc quadrant-defaults
                      (quadrant (:neck (first skel-hist))) 1)]
                (doseq
                    [q quadrants]
                  (overtone/ctl
                   (q insts)
                   :vol
                   (/ (* @master-vol
                         (:quadrant @osc-inst-toggles) (:quadrant @osc-inst-vols)
                         (q @osc-quadrant-toggles) (q @osc-quadrant-vols)
                         (q quadrant-state)) num-uids)))))
            (if (ready skel-hist n)
              (doseq [joint ctl-joints]
                (if (contains? hands joint)
                      (if (= 0 (mod @ctl-counter ctl-mod))
                        (overtone/ctl
                         (joint insts)
                         :freq
                         (let [chord
                               (joint note-chords)
                               index
                               (int (overtone/scale-range
                                     (/ (:sum (:y (joint (skeleton-range skel-hist)))) n)
                                     -500 500
                                     0 (- (count chord) 1)))
                               midi-note
                               (nth chord index)
                               f
                               (overtone/midi->hz midi-note)]
                           f)
                         :vol
                         (overtone/scale-range
                          (math/abs
                           (/ (:sum (:x (joint
                                         (skeleton-range (velocity skel-hist))))) n))
                          0 (* 0.01 (stage-size :x))
                          0 (/ (* @master-vol
                                  (joint @osc-inst-toggles)
                                  (joint @osc-inst-vols)) num-uids))))
                      (overtone/ctl
                 (joint insts)
                 :freq
                 (overtone/scale-range
                  (/ (:sum (:y (joint (skeleton-range skel-hist)))) n)
                  -1000 500
                  (first (joint freq-range))
                  (second (joint freq-range)))
                 :vol
                 (overtone/scale-range
                  (math/abs
                   (/ (:sum (:x (joint
                                 (skeleton-range (velocity skel-hist))))) n))
                  0 (* 0.01 (stage-size :x))
                  0 (/ (* @master-vol
                          (joint @osc-inst-toggles)
                          (joint @osc-inst-vols)) num-uids)))))))
          (doseq
              [[keyword-uid inst] insts]
            (overtone/ctl inst :vol 0)))))
  (swap! ctl-counter inc))

(add-watch keyword-skeletons-hist
           :keyword-skeletons-hist-watcher ctl-sound)

(overtone/stop)
(def server (overtone/osc-server 44100 "openni-overtone"))

(overtone/osc-handle
 server
 "/1/master-vol"
 (fn [msg]
     (reset! master-vol (first (:args msg)))))

(overtone/osc-handle
 server
 "/1/inst-toggles"
 (fn [msg]
   (doseq [i (count ctls)]
     (swap! osc-inst-toggles assoc (nth ctls i) (nth (:args msg) i)))))

(overtone/osc-handle
 server
 "/1/inst-vols"
 (fn [msg]
   (doseq [i (count ctls)]
     (swap! osc-inst-vols assoc (nth ctls i) (nth (:args msg) i)))))

(overtone/osc-handle
 server
 "/1/quadrant-toggles"
 (fn [msg]
   (doseq [i (count quadrants)]
     (swap! osc-quadrant-toggles assoc (nth ctls i) (nth (:args msg) i)))))

(overtone/osc-handle
 server
 "/1/quadrant-vols"
 (fn [msg]
   (doseq [i (count quadrants)]
     (swap! osc-quadrant-vols assoc (nth ctls i) (nth (:args msg) i)))))

(overtone/osc-handle
 server
 "/1/snapshot"
 (fn [msg]
   (swap! phrases cons @keyword-skeletons-hist)))

;; setup osc (manual)
;; change as needed
;;(def client (overtone/osc-client "192.168.1.10" 9000))
;;(set-touchosc-defaults client)
;; Follow instructions in (https://github.com/overtone/overtone/wiki/TouchOSC).
;; (overtone/zero-conf-on)
;; (overtone/zero-conf-off)
;; set default values in TouchOSC
;; tell TouchOSC to send to this server.
