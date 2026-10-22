#!/usr/bin/env roseus

(load "package://auto_dem/euslisp/dielectrometer-manip.l")

(ros::roseus "dielectrometer_manip_node")
(ros::rate 10)  ;; 10Hz


(defun cb (msg)
  (if (send msg :data)
      (progn
        ;; wait for conveyor transoprt
        ;;(print "start dielec manip in 20 sec")
        ;;(unix::usleep (* 20 1000 1000))

        ;; manip
        ;;(pick-polymer-and-measure-40-28-10 :do-measure t :insert-until-contact t)
        ;;(pick-polymer-and-measure-40-28-10 :do-measure t :insert-until-contact nil)

        ;; test
        (measure-40G :do-measure t :insert-until-contact nil)
        )
    ))

(ros::subscribe "/start_dielectrometer_manip" std_msgs::Bool #'cb)


(defun main()
  (warn "Start dielectrometer_manip_node~%")

  ;; generate instance
  (setq *dem* (instance dielectrometer-manip :init))

  ;; draw
  (objects *display-objects*)

  (warn "Waiting for motion flag~%")
  (do-until-key
   (ros::spin-once)
   (x::window-main-one)
   (ros::sleep)
   )
)

(main)
