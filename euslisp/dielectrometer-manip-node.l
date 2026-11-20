#!/usr/bin/env roseus

(load "package://auto_dem/euslisp/dielectrometer-manip.l")

(ros::roseus "dielectrometer_manip_node")
(ros::rate 10)  ;; 10Hz


(defun cb (msg)
  (if (send msg :data)
      (progn
        ;; manip
        (pick-polymer-and-measure-40-28-10 :do-measure t :insert-until-contact t)
        ;;
        ;; for test
        ;; (measure-40G :do-measure nil :insert-until-contact nil)
        ;; (pick-polymer-and-measure-40-28-10 :do-measure nil :insert-until-contact nil)
        ;; (pick-polymer-and-measure-40-28-10 :do-measure t :insert-until-contact nil)
        )
    ))

(ros::subscribe "/start_dielectrometer_manip" std_msgs::Bool #'cb)


(defun main()
  (warn "Start dielectrometer_manip_node~%")

  ;; generate instance
  (generate-demanip-instance) ;; instance and draw

  (warn "Waiting for motion flag~%")
  (do-until-key
   (ros::spin-once)
   (x::window-main-one)
   (ros::sleep)
   )
)

(main)
