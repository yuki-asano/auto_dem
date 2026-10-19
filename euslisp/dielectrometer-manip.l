#!/usr/bin/env roseus

;; todo:
;; - extract common part for manipulation

(load "package://auto_dem/euslisp/load-dielectrometer-exp.l")
(load "package://robot_interface/euslisp/robot-manipulation.l")
(load "package://robot_interface/euslisp/generate-coords.l")

(ros::load-ros-manifest "aet_instrument")
(ros::roseus-add-msgs "franka_gripper")
(ros::advertise "dielectrometer_cmd" aet_instrument::Dielectrometer 1)


(defclass dielectrometer-manip
  :super robot-manipulation-rarm
  :slots (
          ;; general variable
          run  ;; move robot or not
          wue  ;; wait-until-enter during robot motion sequence
          mode
          ;; RUN     : (run t)   and (wue nil)
          ;; RUN_STEP: (run t)   and (wue t)
          ;; SIM     : (run nil) and (wue nil)
          ;; SIM_STEP: (run nil) and (wue t)
          ;;
          ;; Format: tgc-(verb)-(object)-(location)
          ;;     Ex: tgc-pick/place/insert-object-from/to
          tgc-insert-polymer-10G
          tgc-insert-polymer-28G
          tgc-insert-polymer-40G
          ;; wrench at the end-effector
          wrench
          fx
          fy
          fz
          taux
          tauy
          tauz
          ;;
          polymer-thickness
          )
  )

(defmethod dielectrometer-manip
  (:init
   ()
   ;; state
   (setq run t)
   (setq wue nil)
   ;;
   (send self :generate-coords)
   (send self :set-insert-polymer-endcoords)
   ;; subscriber
   (ros::subscribe "/franka_state_controller/F_ext" geometry_msgs::WrenchStamped #'send self :wrench-cb)
   (ros::subscribe "/polymer_evaluation/polymer_thickness" std_msgs::Float32 #'send self :thickness-cb)
   (ros::subscribe "/reset_gripper" std_msgs::Bool #'send self :reset-gripper-cb)  ;; same with press_machine_exp
   (format t "STATE:~%")
   (format t "run: ~a~%" run)
   (format t "wue: ~a~%" wue)
   )
  ;; common
  (:get-run-value
   ()
   run)
  (:get-wue-value
   ()
   wue)
  (:get-mode
   ()
   (format t "[Current mode]: ~a~%" mode)
   )
  (:change-mode
   (run wue)
   (cond ((and run (not wue))
          (setq mode "RUN"))
         ((and run wue)
          (setq mode "RUN_STEP"))
         ((and (not run) (not wue))
          (setq mode "SIM"))
         ((and (not run) wue)
          (setq mode "SIM_STEP"))
         (t
          (warn "mode error~%")))
   )
  (:change-run-mode
   (run-mode)
   (warn "change run mode~%")
   (cond ((equal run-mode t)
          (setq run t))
         ((equal run-mode nil)
          (setq run nil))
         (t
          (warn "arg should be t or nil")))
   (send self :change-mode run wue)
   (send self :get-mode)
   )
  (:change-wue-mode
   (wue-mode)
   (warn "change wue mode~%")
   (cond ((equal wue-mode t)
          (setq wue t))
         ((equal wue-mode nil)
          (setq wue nil))
         (t
          (warn "arg should be t or nil")))
   (send self :change-mode run wue)
   (send self :get-mode)
   )
  ;; common
  (:generate-coords
   ()
   ;; for pick polymer from slider
   (generate-coords tgc-pick-polymer-pre-reach *polymer-gp-manip-coords* #f(-30 -15 40) #f(0   0 0))
   (generate-coords tgc-pick-polymer-reach     *polymer-gp-manip-coords* #f( 6  -15 45) #f(0 -10 0))
   (generate-coords tgc-pick-polymer-pre-leave *polymer-gp-manip-coords* #f( 6  -15 70) #f(0   0 0))
   ;;
   ;; for inserting 10G apparatus
   (generate-coords tgc-insert-polymer-10G-pre-reach *10G-manip-coords* #f( -70 -20  20) #(0 0 0))
   (generate-coords tgc-insert-polymer-10G-reach     *10G-manip-coords* #f( -20 -20  8)  #(0 0 0))
   (generate-coords tgc-insert-polymer-10G-pre-leave *10G-manip-coords* #f( -70 -20  10) #(0 0 0))
   ;;
   ;; for inserting 28G apparatus
   (generate-coords tgc-insert-polymer-28G-pre-reach *28G-manip-coords* #f( -100  0  10) #(0 0 0))
   (generate-coords tgc-insert-polymer-28G-reach     *28G-manip-coords* #f( -60   0   0) #(0 0 -4))
   (generate-coords tgc-insert-polymer-28G-pre-leave *28G-manip-coords* #f( -100  0  10) #(0 0 0))
   ;;
   ;; for inserting 40G apparatus
   (generate-coords tgc-insert-polymer-40G-pre-reach *40G-manip-coords* #f( -50 -100 50) #(0 0 0))
   (generate-coords tgc-insert-polymer-40G-reach     *40G-manip-coords* #f( -10 -120 50) #(0 0 3))
   (generate-coords tgc-insert-polymer-40G-pre-leave *40G-manip-coords* #f( -50 -100 50) #(0 0 0))
   )
  ;;
  ;; end-coords settings. use :move-to :world method to overwrite coordinates
  (:set-endcoords-button-upside
   ()
   (send *robot* :rarm :end-coords :move-to
         (make-cascoords :pos (send *robot* :rarm :end-coords :worldpos)
                         :rot (rotate-matrix
                               (send *robot* :rarm :end-coords :worldrot) (deg2rad -90) :x nil))
         :world))
  (:set-insert-polymer-endcoords
   ()
   ;; gripper direction is important.
   ;; The button on the gripper for interactive mode should be upside.
   ;; If downside, joint5 is close to limit and vibration tends to occurs.
   (send self :set-endcoords-button-upside))
  ;;
  ;; base poses
  (:reset-pose
   (&key (time 5000))
   (send *robot* :reset-pose)
   (send *irtviewer* :draw-objects)
   (when run
     (send *ri* :angle-vector (send *robot* :angle-vector) time)
     (send *ri* :wait-interpolation)))
  ;;
  (:pick-polymer-init-pose
   (&key (time 8000))
   ;; generation
   ;; (send *robot* :reset-pose)
   ;; (generate-coords tgc-pick-polymer-init *polymer-gp-manip-coords* #f(-100 0 60) #(0 0 0))
   ;; (send *dem* :solve-ik-rarm :tgc tgc-pick-polymer-init :ra nil) ;; at first, solve ik with rotation-axis nil
   ;; (send *dem* :solve-ik-rarm :tgc tgc-pick-polymer-init :ra t)
   (send *robot* :angle-vector
         #f(110.801 66.0939 -49.5549 -101.111 -74.2137 131.909 107.487)
         )
   (send *irtviewer* :draw-objects)
   (when run
     (send *ri* :angle-vector (send *robot* :angle-vector) time)
     (send *ri* :wait-interpolation)))
  ;;
  (:10G-init-pose
   (&key (time 8000))
   ;; generation
   ;; (send *robot* :reset-pose)
   ;; (generate-coords tgc-insert-polymer-10G-init *10G-manip-coords* #f(-100 -30 20) #(0 0 0))
   ;; (send *dem* :solve-ik-rarm :tgc tgc-insert-polymer-10G-init :ra nil) ;; at first, solve ik with rotation-axis nil
   ;; (send *dem* :solve-ik-rarm :tgc tgc-insert-polymer-10G-init :ra t)
   (send *robot* :angle-vector
         ;; - bottun upside pose. button downside pose is bad.
         ;; - j0 should be minus.
         #f(-84.6092 60.6322 54.7851 -139.455 -37.6375 241.176 -82.0384)
         )
   (send *irtviewer* :draw-objects)
   (when run
     (send *ri* :angle-vector (send *robot* :angle-vector) time)
     (send *ri* :wait-interpolation)))
  (:28G-init-pose
   (&key (time 8000))
   ;; generation
   ;; (send *dem* :10G-init-pose)  ;; not reset-pose
   ;; (generate-coords tgc-insert-polymer-28G-init *28G-manip-coords* #f(-100 0 20) #(0 0 0))
   ;; (send *dem* :solve-ik-rarm :tgc tgc-insert-polymer-28G-init :ra nil) ;; at first, solve ik with rotation-axis nil
   ;; (send *dem* :solve-ik-rarm :tgc tgc-insert-polymer-28G-init :ra t)
   (send *robot* :angle-vector
         ;; - bottun upside pose. button downside pose is bad.
         ;; - from 10G-init-pose. from reset pose is bad
         ;; - j0 should be minus.
         #f(-79.0308 86.7057 74.8923 -143.844 -15.7306 241.181 -122.942)
         )
   (send *irtviewer* :draw-objects)
   (when run
     (send *ri* :angle-vector (send *robot* :angle-vector) time)
     (send *ri* :wait-interpolation)))
  (:40G-init-pose
   (&key (time 8000))
   ;; generation
   ;; (send *robot* :reset-pose)
   ;; (generate-coords tgc-insert-polymer-40G-init *40G-manip-coords* #f(-100 0 0) #(0 0 0))
   ;; (send *dem* :solve-ik-rarm :tgc tgc-insert-polymer-40G-init :ra nil) ;; rotation-axis nil first
   ;; (send *dem* :solve-ik-rarm :tgc tgc-insert-polymer-40G-init :ra t)
   (send *robot* :angle-vector
         ;; - button upside pose
         ;; - from reset-pose. from 28G-init-pose is bad (j4 is plus).
         ;; -- if this case, transition movement is too large and collide the table.
         ;; - j0 should be plus. j4 should be minus.
         #f(100.885 41.409 -70.7532 -143.386 -142.094 128.702 154.097)
         )
   (send *irtviewer* :draw-objects)
   (when run
     (send *ri* :angle-vector (send *robot* :angle-vector) time)
     (send *ri* :wait-interpolation)))
  ;;
  ;; callback
  (:wrench-cb
   (msg)
   (setq fx (send msg :wrench :force :x))
   (setq fy (send msg :wrench :force :y))
   (setq fz (send msg :wrench :force :z))
   (setq taux (send msg :wrench :torque :x))
   (setq tauy (send msg :wrench :torque :y))
   (setq tauz (send msg :wrench :torque :z))
   (setq wrench (float-vector fx fy fz taux tauy tauz))
   )
  (:thickness-cb
   (msg)
   (setq polymer-thickness (send msg :data))
   )
  (:get-polymer-thickness
   ()
   (let (_thickness)
     (ros::spin-once)
     (ros::sleep)

     (if polymer-thickness
         (setq _thickness polymer-thickness)
       (setq _thickness 0.3))
     _thickness)
   )
  (:move-until-contact
   (&key (diff-vec #f(0 0 -1))
         (wrench-mask-vec #i(0 0 1 0 0 0))
         (wrench-limit-vec #f(0.0 0.0 0.0 0.0 0.0 0.0)))
   ;; arg:
   ;; - diff-vec        : translation distance in a cycle (world coords)
   ;; - wrench-mask-vec : monitored wrench component should be 1
   ;; - wrench-limit-vec: limit value of wrench component [N]/[Nm]
   (let (ec-now ec-pos-next ec-rot-next tgc-next wrench-masked (i 0))
     (format t "-----------------------------~%")
     (format t "Start move-until-contact mode~%")
     (if (> (norm diff-vec) 1.0)
         (progn
           (setq diff-vec #f(0 0 0))
           (format t "diff-vec set to #f(0 0 0) since it was too large")))
     (while t
       (setq ec-now (send *robot* :rarm :end-coords))
       (setq ec-pos-next (v+ (send ec-now :worldpos) diff-vec))
       (setq ec-rot-next (send ec-now :worldrot))
       (setq tgc-next (make-coords :pos ec-pos-next :rot ec-rot-next))
       (send self :solve-ik-rarm :tgc tgc-next :wue nil :run run :time 500 :cartesian t)  ;; motion speed is tuned as 1mm translation with 0.5 sec. enough short cycle time is important.
       ;; check the wrench at the end-effector
       (ros::spin-once)
       (setq wrench-masked wrench)
       (dotimes (x (length wrench-mask-vec))
         (if (equal (elt wrench-mask-vec x) 0)
             (setf (elt wrench-masked x) 0)))
       (format t "wrench-masked: ~a~%" wrench-masked)
       (dotimes (x (length wrench-limit-vec))
         (if (> (elt wrench-masked x) (elt wrench-limit-vec x))
             (progn
               (format t "~a-th wrench over the limit~%" x)
               (format t "End move-until-contact mode~%")
               (format t "---------------------------~%")
               (return-from :move-until-contact))))
       ;; print
       (format t "cycle   : ~A~%" i)
       (format t "tgc-next: ~A~%" tgc-next)
       (setq i (1+ i))
       )))
  ;; for grasp
  (:grasp-film
   ()
   (send self :grasp-tool :width 0.0003 :inner 0.0005 :outer 0.001))    ;; -0.2mm < 0.3mm < 1.3mm
  (:release-film
   ()
   (send self :release-tool))
  ;; for pick & place
  (:pick-polymer-film
   (pick-obj coords-list time-list
             &key (grasp-timing nil)
             (skip-gripper nil)
             (cartesian nil))
   (send self :pick-object-base
         pick-obj coords-list time-list
         :grasp-timing grasp-timing
         :grasp-width  0.0003
         :grasp-time   500
         :grasp-inner  0.0005
         :grasp-outer  0.001
         :grasp-effort 10
         :skip-gripper skip-gripper
         :cartesian cartesian
         ))
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; dielectrometer manipulation
  ;; - Format: verb-object-location
  ;;
  ;; pick polymer at first
  (:pick-polymer-from-slider
   (&key (init-pose-time 10000)
         (end-pose-time 10000)
         )
   ;; init
   (send self :pick-polymer-init-pose :time init-pose-time)
   ;; pick
   (send self :pick-polymer-film
         nil
         (list tgc-pick-polymer-pre-reach
               tgc-pick-polymer-reach
               tgc-pick-polymer-pre-leave)
         (list 6000 6000 3000)
         :grasp-timing 2
         :cartesian t)
   ;; end-pose
   (send self :pick-polymer-init-pose :time end-pose-time)
   )
  ;;
  ;; dielectrometer insertion and measurement
  (:insert-polymer-10G
   (&key (init-pose-time 10000)
         (end-pose-time 10000)
         (thickness 0.3)
         (sample-name "sample")
         (file-path "test.csv")
         (do-measure t)
         (insert-until-contact nil)
         )
   ;; init
   (send self :10G-init-pose :time init-pose-time)
   ;; place
   (if insert-until-contact
       (progn
         (send self :solve-ik-rarm :tgc tgc-insert-polymer-10G-pre-reach :wue wue :run run :time 6000 :cartesian t)
         (send self :solve-ik-rarm :tgc tgc-insert-polymer-10G-reach :wue wue :run run :time 6000 :cartesian t)
         (send self :move-until-contact :diff-vec #f(1 0 0) :wrench-mask-vec #i(0 0 1 0 0 0) :wrench-limit-vec #f(0.0 0.0 5.0 0.0 0.0 0.0))
         (unix::usleep (* 1000 1000))  ;; release polymer after stopped and stable condition
         (send *ri* :send-gripper-move-action :rarm 0.005 0.01) ;; open 5mm with 1cm/s slowly. gripper may collide to the dielectrometer when using stop-grasp (motion is too large)
         )
     (progn
       (send self :place-object-base
             nil
             (list tgc-insert-polymer-10G-pre-reach
                   tgc-insert-polymer-10G-reach)
             (list 6000 6000)
             :release-timing 2
             :cartesian t)
       ))
   (unix::usleep (* 1000 1000))  ;; wait a second after sample release
   ;; measure
   (when do-measure
     (setq thickness (send self :get-polymer-thickness))
     (format t "[Measurement] Waiting for finish~%")
     (send self :measure-target-10G :thickness thickness) ;; publish
     (send self :calc-and-save-10G :thickness thickness :sample-name sample-name :file-path file-path))
   ;; pick
   (send self :pick-polymer-film
         nil
         (list tgc-insert-polymer-10G-pre-leave)
         (list 3000)
         :grasp-timing 0
         :cartesian t)
   ;; end-pose
   (send self :10G-init-pose :time end-pose-time)
   )
  (:insert-polymer-28G
   (&key (init-pose-time 10000)
         (end-pose-time 10000)
         (thickness 0.3)
         (sample-name "sample")
         (file-path "test.csv")
         (do-measure t)
         (insert-until-contact nil)
         )
   ;; init
   (send self :28G-init-pose :time init-pose-time)
   ;; place
   (if insert-until-contact
       (progn
         (send self :solve-ik-rarm :tgc tgc-insert-polymer-28G-pre-reach :wue wue :run run :time 6000 :cartesian t)
         (send self :solve-ik-rarm :tgc tgc-insert-polymer-28G-reach :wue wue :run run :time 6000 :cartesian t)
         (send self :move-until-contact :diff-vec #f(1 0 0) :wrench-mask-vec #i(0 0 1 0 0 0) :wrench-limit-vec #f(0.0 0.0 5.0 0.0 0.0 0.0))
         (unix::usleep (* 1000 1000))  ;; same with 10G
         (send *ri* :send-gripper-move-action :rarm 0.005 0.01) ;; same with 10G
         )
     (progn
       (send self :place-object-base
             nil
             (list tgc-insert-polymer-28G-pre-reach
                   tgc-insert-polymer-28G-reach)
             (list 6000 6000)
             :release-timing 2
             :cartesian t)
       ))
   (unix::usleep (* 1000 1000))  ;; wait a second after sample release
   ;; measure
   (when do-measure
     (format t "[Measurement] Waiting for finish~%")
     (send self :measure-target-28G :thickness thickness) ;; publish
     (send self :calc-and-save-28G :thickness thickness :sample-name sample-name :file-path file-path))
   ;; pick
   (send self :pick-polymer-film
         nil
         (list tgc-insert-polymer-28G-pre-leave)
         (list 3000)
         :grasp-timing 0
         :cartesian t)
   ;; end-pose
   (send self :28G-init-pose :time end-pose-time)
   )
  (:insert-polymer-40G
   (&key (init-pose-time 10000)
         (end-pose-time 10000)
         (thickness 0.3)
         (sample-name "sample")
         (file-path "test.csv")
         (do-measure t)
         (insert-until-contact nil)
         )
   ;; init
   (send self :40G-init-pose :time init-pose-time)
   ;; place
   (if insert-until-contact
       (progn
         (send self :solve-ik-rarm :tgc tgc-insert-polymer-40G-pre-reach :wue wue :run run :time 6000 :cartesian t)
         (send self :solve-ik-rarm :tgc tgc-insert-polymer-40G-reach :wue wue :run run :time 6000 :cartesian t)
         (send self :move-until-contact :diff-vec #f(1 0 0) :wrench-mask-vec #i(0 0 1 0 0 0) :wrench-limit-vec #f(0.0 0.0 5.0 0.0 0.0 0.0))
         (unix::usleep (* 1000 1000))  ;; same with 10G
         (send *ri* :send-gripper-move-action :rarm 0.005 0.01) ;; same with 10G
         )
     (progn
       (send self :place-object-base
             nil
             (list tgc-insert-polymer-40G-pre-reach
                   tgc-insert-polymer-40G-reach)
             (list 6000 6000)
             :release-timing 2
             :cartesian t)
       ))
   (unix::usleep (* 1000 1000))  ;; wait a second after sample release
   ;; measure
   (when do-measure
     (format t "[Measurement] Waiting for finish~%")
     (send self :measure-target-40G :thickness thickness) ;; publish
     (send self :calc-and-save-40G :thickness thickness :sample-name sample-name :file-path file-path))
   ;; pick
   (send self :pick-polymer-film
         nil
         (list tgc-insert-polymer-40G-pre-leave)
         (list 3000)
         :grasp-timing 0
         :cartesian t)
   ;; end-pose
   (send self :40G-init-pose :time end-pose-time)
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;
  ;; dielectrometer_cmd
  (:measure-cmd-base
   ()
   (let (_msg)
     (setq _msg (instance aet_instrument::Dielectrometer :init))
     (send _msg :do_id 0)  ;; cmd_id (0:measure blank, 1: measure target, 2: calc and save)
     (send _msg :f 0)      ;; Frequency of the dielectrometer (0: 10GHz, 1: 28GHz, 2: 40GHz)
     (send _msg :g 2.5)    ;; Guess value of epsilon
     (send _msg :t 0.3)    ;; Thickness of the sample polymer
     (send _msg :switch t)
     (send _msg :name "test")
     (send _msg :file "test")
     _msg))
  ;; measure blank
  (:measure-blank-base
   (&key freq)
   (let (msg)
     (setq msg (send self :measure-cmd-base))
     (send msg :do_id 0)
     (send msg :f freq)
     (ros::publish "/dielectrometer_cmd" msg)
     (unix::usleep (* 20 1000 1000))
     ))
  (:measure-blank-10G
   ()
   (send self :measure-blank-base :freq 0))
  (:measure-blank-28G
   ()
   (send self :measure-blank-base :freq 1))
  (:measure-blank-40G
   ()
   (send self :measure-blank-base :freq 2))
  ;; measure target
  (:measure-target-base
   (&key freq (thickness 0.3))
   (let (msg)
     (setq msg (send self :measure-cmd-base))
     (send msg :do_id 1)
     (send msg :f freq)
     (send msg :t (send self :get-polymer-thickness))
     (send msg :t thickness)
     (ros::publish "/dielectrometer_cmd" msg)
     (unix::usleep (* 20 1000 1000))
     ))
  (:measure-target-10G
   (&key (thickness 0.3))
   (setq thickness (send self :get-polymer-thickness))
   (send self :measure-target-base :freq 0 :thickness thickness))
  (:measure-target-28G
   (&key (thickness 0.3))
   (setq thickness (send self :get-polymer-thickness))
   (send self :measure-target-base :freq 1 :thickness thickness))
  (:measure-target-40G
   (&key (thickness 0.3))
   (setq thickness (send self :get-polymer-thickness))
   (send self :measure-target-base :freq 2 :thickness thickness))
  (:calc-and-save-base
   (&key freq
         (thickness 0.3)
         (sample-name "sample")
         (file-path "C:/Users/test/Desktop/test_file.csv")  ;; Windows
         )
   (let (msg)
     (setq msg (send self :measure-cmd-base))
     (send msg :do_id 2)
     (send msg :f freq)
     (send msg :t thickness)
     (send msg :name sample-name)
     (send msg :file file-path)
     (ros::publish "/dielectrometer_cmd" msg)
     (unix::usleep (* 2000 1000))
     ))
  (:calc-and-save-10G
   (&key thickness sample-name file-path)
   (setq thickness (send self :get-polymer-thickness))
   (send self :calc-and-save-base :freq 0 :thickness thickness :sample-name sample-name :file-path file-path))
  (:calc-and-save-28G
   (&key thickness sample-name file-path)
   (setq thickness (send self :get-polymer-thickness))
   (send self :calc-and-save-base :freq 1 :thickness thickness :sample-name sample-name :file-path file-path))
  (:calc-and-save-40G
   (&key thickness sample-name file-path)
   (setq thickness (send self :get-polymer-thickness))
   (send self :calc-and-save-base :freq 2 :thickness thickness :sample-name sample-name :file-path file-path))
  )  ;; dielectrometer-manip


;; settings
(setq *dem* (instance dielectrometer-manip :init)) ;; dem: dielectrometer-manip
(send *robot* :reset-pose)
(send *irtviewer* :draw-objects)


;; functions
(defun pick-polymer-from-slider
    (&key (start-reset-pose t)
          (start-reset-pose-time 7000)
          (end-reset-pose t)
          (end-reset-pose-time 10000)
          )
  (when start-reset-pose
    (send *dem* :reset-pose :time start-reset-pose-time))
  (send *dem* :pick-polymer-from-slider :init-pose-time 10000 :end-pose-time 5000)
  (when end-reset-pose
    (send *dem* :reset-pose :time end-reset-pose-time))
  )

(defun measure-test
    ()
  (format t "measuring 10G blank~%")
  (send *dem* :measure-blank-10G)
  (format t "measuring 28G blank~%")
  (send *dem* :measure-blank-28G)
  (format t "measuring 40G blank~%")
  (send *dem* :measure-blank-40G)
  )

(defun measure-10G
    (&key (thickness 0.3)
          (sample-name "sample")
          (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
          (start-reset-pose t)
          (start-reset-pose-time 5000)
          (end-reset-pose t)
          (end-reset-pose-time 5000)
          (do-measure t)
          (insert-until-contact nil)
          )
  (when start-reset-pose
    (send *dem* :reset-pose :time start-reset-pose-time))
  (when do-measure
    (send *dem* :measure-blank-10G))
  (send *dem* :insert-polymer-10G :init-pose-time 10000 :end-pose-time 5000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact)
  (when end-reset-pose
    (send *dem* :reset-pose :time end-reset-pose-time))
  )

(defun measure-28G
    (&key (thickness 0.3)
          (sample-name "sample")
          (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
          (start-reset-pose t)
          (start-reset-pose-time 7000)
          (end-reset-pose t)
          (end-reset-pose-time 8000)
          (do-measure t)
          (insert-until-contact nil)
          )
  (when start-reset-pose
    (send *dem* :reset-pose :time start-reset-pose-time))
  (when do-measure
    (send *dem* :measure-blank-28G))
  (send *dem* :insert-polymer-28G :init-pose-time 7000 :end-pose-time 5000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact)
  (when end-reset-pose
    (send *dem* :reset-pose :time end-reset-pose-time))
  )

(defun measure-40G
    (&key (thickness 0.3)
          (sample-name "sample")
          (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
          (start-reset-pose t)
          (start-reset-pose-time 7000)
          (end-reset-pose t)
          (end-reset-pose-time 10000)
          (do-measure t)
          (insert-until-contact nil)
          )
  (when start-reset-pose
    (send *dem* :reset-pose :time start-reset-pose-time))
  (when do-measure
    (send *dem* :measure-blank-40G))
  (send *dem* :insert-polymer-40G :init-pose-time 10000 :end-pose-time 5000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact)
  (when end-reset-pose
    (send *dem* :reset-pose :time end-reset-pose-time))
  )

(defun measure-10-28-40
    (&key
     (thickness 0.3)
     (sample-name "sample")
     (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
     (do-measure t)
     (insert-until-contact nil)
     )
  ;; 10G
  (measure-10G :thickness thickness :sample-name sample-name :file-path file-path :start-reset-pose t :start-reset-pose-time 5000 :end-reset-pose nil :do-measure do-measure :insert-until-contact insert-until-contact)
  ;; 28G
  (measure-28G :thickness thickness :sample-name sample-name :file-path file-path :start-reset-pose nil :end-reset-pose t :end-reset-pose-time 8000 :do-measure do-measure :insert-until-contact insert-until-contact)  ;; end-reset t: to avoid collision to table. trajectory is wider due to large pose changes
  ;; 40G
  (measure-40G :thickness thickness :sample-name sample-name :file-path file-path :start-reset-pose nil :end-reset-pose t :end-reset-pose-time 10000 :do-measure do-measure :insert-until-contact insert-until-contact)
  )

;; measurement benchmark
(defun measure-10-28-40-ntimes-each
    (&key
     (measure-times 3)
     (thickness 0.3)
     (sample-name "sample")
     (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
     (do-measure t)
     (insert-until-contact nil)
     )
  ;; initial pose
  (send *dem* :reset-pose :time 5000)
  ;; 10G
  (send *dem* :10G-init-pose :time 8000)
  (dotimes (n measure-times)
    (when do-measure
      (send *dem* :measure-blank-10G))
    (send *dem* :insert-polymer-10G :init-pose-time 1000 :end-pose-time 3000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact))
  ;; 28G
  (send *dem* :28G-init-pose :time 3000)
  (dotimes (n measure-times)
    (when do-measure
      (send *dem* :measure-blank-28G))
    (send *dem* :insert-polymer-28G :init-pose-time 1000 :end-pose-time 3000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact))
  (send *dem* :reset-pose :time 8000) ;; to avoid collision to table. trajectory is wider due to large pose changes
  ;; 40G
  (send *dem* :40G-init-pose :time 8000)
  (dotimes (n measure-times)
    (when do-measure
      (send *dem* :measure-blank-40G))
    (send *dem* :insert-polymer-40G :init-pose-time 1000 :end-pose-time 3000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact))
  (send *dem* :reset-pose :time 8000)
  )

(defun pick-polymer-and-measure-40-28-10
    (&key
     (thickness 0.3)
     (sample-name "sample")
     (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
     (do-measure nil)
     (insert-until-contact nil)
     )
  ;; pick polymer
  (pick-polymer-from-slider :start-reset-pose t :start-reset-pose-time 7000 :end-reset-pose nil)
  ;; 40G
  (measure-40G :thickness thickness :sample-name sample-name :file-path file-path :start-reset-pose nil :end-reset-pose t :end-reset-pose-time 10000 :do-measure do-measure :insert-until-contact insert-until-contact)  ;; end-reset t: to avoid collision to table. trajectory is wider due to large pose changes
  ;; 28G
  (measure-28G :thickness thickness :sample-name sample-name :file-path file-path :start-reset-pose nil :end-reset-pose nil :do-measure do-measure :insert-until-contact insert-until-contact)
  ;; 10G
  (measure-10G :thickness thickness :sample-name sample-name :file-path file-path :start-reset-pose nil :end-reset-pose t :do-measure do-measure :insert-until-contact insert-until-contact)
  )

(defun pick-polymer-and-measure-40-28-10-ntimes-each
    (&key
     (measure-times 3)
     (thickness 0.3)
     (sample-name "sample")
     (file-path "C:/Users/test/Desktop/asano/test_file.csv")  ;; Windows
     (do-measure t)
     (insert-until-contact nil)
     )
  ;; pick polymer
  (pick-polymer-from-slider :start-reset-pose t :start-reset-pose-time 7000 :end-reset-pose nil)
  ;; 40G
  (send *dem* :40G-init-pose :time 8000)
  (dotimes (n measure-times)
    (when do-measure (send *dem* :measure-blank-40G))
    (send *dem* :insert-polymer-40G :init-pose-time 1000 :end-pose-time 3000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact))
  (send *dem* :reset-pose :time 8000)  ;; large motion at 28G-40G transition
  ;; 28G
  (send *dem* :28G-init-pose :time 8000)
  (dotimes (n measure-times)
    (when do-measure (send *dem* :measure-blank-28G))
    (send *dem* :insert-polymer-28G :init-pose-time 1000 :end-pose-time 3000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact))
  ;; no reset-pose here
  ;; 10G
  (send *dem* :10G-init-pose :time 3000)
  (dotimes (n measure-times)
    (when do-measure (send *dem* :measure-blank-10G))
    (send *dem* :insert-polymer-10G :init-pose-time 1000 :end-pose-time 3000 :thickness thickness :sample-name sample-name :file-path file-path :do-measure do-measure :insert-until-contact insert-until-contact))
  (send *dem* :reset-pose :time 8000)
  )


(warn "(send *dem* :change-run-mode nil)~%")
(warn "(send *dem* :change-wue-mode nil)~%")
(warn "(pick-polymer-and-measure-40-28-10 :do-measure t :insert-until-contact t)~%")
(warn "(pick-polymer-and-measure-40-28-10-ntimes-each :measure-times 3 :do-measure t :insert-until-contact t)~%")
