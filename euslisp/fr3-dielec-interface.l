(require :robot-interface "package://panda_eus/euslisp/fr3-interface.l")
(require :fr3-dielec "package://auto_dem/models/fr3-dielec.l")

(defclass fr3-dielec-robot-interface
  :super fr3-robot-interface
  :slots ()
  )

(defmethod fr3-dielec-robot-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (send-super* :init args)
   )
  (:default-controller  ;; change controller to effort as default
   ()
   (list
    (list
     (cons :controller-action "/effort_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state  "/effort_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name)))))
  )

(defun fr3-dielec-init ()
  (setq *ri* (instance fr3-dielec-robot-interface :init))
  (setq *robot* (fr3_dielec))
  )
