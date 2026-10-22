#!/usr/bin/env roseus
;; RPC node that exposes auto_dem's manipulation to the embrio HTTP server (auto_dem_http_server.py).
;; One roseus/StringString service: the request is an S-expression (name :key value ...), the reply is JSON.
;; A custom srv would touch the package build, which 3rd_party/ must not do.
;;
;; Service /embrio/auto_dem/call (roseus/StringString)
;;   (ping)
;;   (pose)                                   -> joints [deg] x7, position [x y z roll pitch yaw] (mm/deg), gripper_mm
;;   (move_joints :joints (7 values) :speed deg/s)
;;   (move_position :position (6 values) :speed mm/s :step_mm mm)   straight line, piecewise IK
;;   (gripper_move :position mm :speed mm/s :force N)              -> achieved mm
;;   (gripper_position)
;;   (run_measurement :thickness mm :sample_name s :file_path s :do_measure t :insert_until_contact t :measure_times n)
;; Reply: {"ok": true, "result": ...} | {"ok": false, "error": "..."}
;; Topic /embrio/auto_dem/abort (std_msgs/Empty): cancels the running motion and unwinds the running handler.
;; Pose convention: R = Rz(yaw) Ry(pitch) Rx(roll), degrees, as the embrio arm drivers expect.

(load "package://auto_dem/euslisp/dielectrometer-manip.l")
(ros::roseus-add-srvs "roseus")
(ros::roseus-add-msgs "std_msgs")

(defvar *embrio-busy* nil)
(defvar *embrio-arm-joint-count* 7)
(defvar *embrio-joint-tolerance-deg* 1.0)
(defvar *embrio-position-tolerance-mm* 2.0)
(defvar *embrio-rotation-tolerance-deg* 2.0)
(defvar *embrio-gripper-open-max-m* 0.08)

;; Own TCP on the hand link: the upstream end-coords is re-oriented for the dielectrometer
;; insertion, so the embrio side keeps a plain hand-axis frame instead.
(setq *embrio-tcp*
      (make-cascoords :coords (send (send *robot* :fr3_hand_lk) :copy-worldcoords) :name "embrio_tcp"))
(send *embrio-tcp* :translate (float-vector 0 0 103.4))
(send (send *robot* :fr3_hand_lk) :assoc *embrio-tcp*)

;; --- JSON encoder (numbers, strings, t / :false, nil as null, lists, alists with string keys) ---

(defun embrio-json-string (s)
  (let ((out (make-string-output-stream)))
    (write-byte #\" out)
    (dotimes (i (length s))
      (let ((c (char s i)))
        (cond ((= c #\") (princ "\\\"" out))
              ((= c #\\) (princ "\\\\" out))
              ((= c 10) (princ "\\n" out))
              ((< c 32) (write-byte #\space out))
              (t (write-byte c out)))))
    (write-byte #\" out)
    (get-output-stream-string out)))

(defun embrio-json-object-p (v)
  (and (consp v)
       (every #'(lambda (e) (and (consp e) (stringp (car e)))) v)))

(defun embrio-json (v)
  (cond ((null v) "null")
        ((eq v t) "true")
        ((eq v :false) "false")
        ((integerp v) (format nil "~d" v))
        ((floatp v) (format nil "~a" v))
        ((stringp v) (embrio-json-string v))
        ((symbolp v) (embrio-json-string (string-downcase (symbol-name v))))
        ((vectorp v) (embrio-json (coerce v cons)))
        ((embrio-json-object-p v)
         (format nil "{~a}"
                 (reduce #'(lambda (a b) (format nil "~a,~a" a b))
                         (mapcar #'(lambda (e) (format nil "~a:~a" (embrio-json-string (car e)) (embrio-json (cdr e)))) v))))
        ((listp v)
         (format nil "[~a]"
                 (reduce #'(lambda (a b) (format nil "~a,~a" a b)) (mapcar #'embrio-json v))))
        (t (embrio-json-string (format nil "~a" v)))))

(defun embrio-bool (v) (if v t :false))

;; --- errors: the default handler would drop into the debugger and freeze the node ---

(defvar *embrio-in-call* nil)

;; Outside a call there is no :embrio-error catcher, so fall back to roseus' print-and-exit.
(defun embrio-error-handler (code msg1 form &optional (msg2))
  (unless *embrio-in-call*
    (ros::roseus-error code msg1 form msg2))
  (throw :embrio-error
         (list :error (cond ((null msg2) msg1)
                            ((zerop (length msg1)) (format nil "~a" msg2))
                            (t (format nil "~a ~a" msg1 msg2))))))

(lisp::install-error-handler 'embrio-error-handler)

;; --- request parsing ---

(defun embrio-parse (text)
  (when (position #\# text)
    (error "request must not contain #"))
  (let ((form (read-from-string text)))
    (unless (and (consp form) (symbolp (car form)))
      (error "request must be (name :key value ...)"))
    form))

(defun embrio-arg (args key &optional (default nil default-p))
  (let ((m (member key args)))
    (cond (m (cadr m))
          (default-p default)
          (t (error "missing argument ~a" key)))))

(defun embrio-number (v what)
  (unless (numberp v) (error "~a must be a number" what))
  (float v))

(defun embrio-positive (v what)
  (let ((n (embrio-number v what)))
    (unless (> n 0.0) (error "~a must be > 0" what))
    n))

(defun embrio-float-list (v count what)
  (unless (and (listp v) (= (length v) count) (every #'numberp v))
    (error "~a must be a list of ~a numbers" what count))
  (mapcar #'float v))

;; --- robot state ---

(defun embrio-sync-model ()
  (let ((av (send *ri* :state :potentio-vector)))
    (send *robot* :angle-vector av)
    av))

(defun embrio-arm-joints (av)
  (let (out)
    (dotimes (i *embrio-arm-joint-count*) (push (elt av i) out))
    (nreverse out)))

(defun embrio-gripper-mm ()
  (* 1000.0 (send *ri* :gripper :rarm :position)))

(defun embrio-pose-of (coords)
  (let ((pos (send coords :worldpos))
        (ypr (car (send coords :rpy-angle))))
    (list (elt pos 0) (elt pos 1) (elt pos 2)
          (rad2deg (elt ypr 2)) (rad2deg (elt ypr 1)) (rad2deg (elt ypr 0)))))

(defun embrio-coords-from-pose (pose)
  (make-coords :pos (float-vector (elt pose 0) (elt pose 1) (elt pose 2))
               :rot (rpy-matrix (deg2rad (elt pose 5)) (deg2rad (elt pose 4)) (deg2rad (elt pose 3)))))

(defun embrio-check-joint-limits (joints)
  (let ((jl (send *robot* :rarm :joint-list)))
    (dotimes (i *embrio-arm-joint-count*)
      (let ((j (elt jl i)) (v (elt joints i)))
        (unless (<= (send j :min-angle) v (send j :max-angle))
          (error "joint~a=~a out of range [~a, ~a] deg" (1+ i) v (send j :min-angle) (send j :max-angle)))))))

;; --- handlers ---

(defun embrio-ping (args)
  (list (cons "node" "auto_dem_embrio_manip")))

(defun embrio-pose (args)
  (let ((av (embrio-sync-model)))
    (list (cons "joints" (embrio-arm-joints av))
          (cons "position" (embrio-pose-of *embrio-tcp*))
          (cons "gripper_mm" (embrio-gripper-mm)))))

(defun embrio-move-joints (args)
  (let* ((joints (embrio-float-list (embrio-arg args :joints) *embrio-arm-joint-count* "joints"))
         (speed (embrio-positive (embrio-arg args :speed) "speed"))
         (av (embrio-sync-model))
         (target (copy-seq av))
         (dmax 0.0))
    (embrio-check-joint-limits joints)
    (dotimes (i *embrio-arm-joint-count*)
      (setf (elt target i) (elt joints i))
      (setq dmax (max dmax (abs (- (elt joints i) (elt av i))))))
    (let ((tm (max 500 (round (* 1000.0 (/ dmax speed))))))
      (send *robot* :angle-vector target)
      (send *ri* :angle-vector target tm)
      (send *ri* :wait-interpolation))
    (let ((reached (embrio-arm-joints (embrio-sync-model))))
      (dotimes (i *embrio-arm-joint-count*)
        (when (> (abs (- (elt reached i) (elt joints i))) *embrio-joint-tolerance-deg*)
          (error "joint~a stopped at ~a deg, target ~a" (1+ i) (elt reached i) (elt joints i))))
      (list (cons "joints" reached)))))

(defun embrio-move-position (args)
  (let* ((pose (embrio-float-list (embrio-arg args :position) 6 "position"))
         (speed (embrio-positive (embrio-arg args :speed) "speed"))
         (step (embrio-positive (embrio-arg args :step_mm 5.0) "step_mm"))
         (av (embrio-sync-model))
         (start (send *embrio-tcp* :copy-worldcoords))
         (goal (embrio-coords-from-pose pose))
         (dist (norm (v- (send goal :worldpos) (send start :worldpos))))
         (n (max 1 (ceiling (/ dist step))))
         (segment-ms (max 100 (round (* 1000.0 (/ (/ dist n) speed)))))
         (avs nil) (tms nil))
    ;; Every waypoint is solved before anything moves, so an unreachable line leaves the arm where it is.
    (dotimes (k n)
      (let ((tgt (midcoords (/ (float (1+ k)) n) start goal)))
        (unless (send *robot* :rarm :inverse-kinematics tgt :move-target *embrio-tcp* :rotation-axis t :warnp nil)
          (send *robot* :angle-vector av)
          (error "ik failed at waypoint ~a of ~a" (1+ k) n))
        (push (copy-seq (send *robot* :angle-vector)) avs)
        (push segment-ms tms)))
    (send *ri* :angle-vector-sequence (nreverse avs) (nreverse tms))
    (send *ri* :wait-interpolation)
    (embrio-sync-model)
    (let ((dp (norm (send goal :difference-position *embrio-tcp*)))
          (dr (rad2deg (norm (send goal :difference-rotation *embrio-tcp*)))))
      (when (or (> dp *embrio-position-tolerance-mm*) (> dr *embrio-rotation-tolerance-deg*))
        (error "arm stopped ~a mm / ~a deg short of the target" dp dr))
      (list (cons "position" (embrio-pose-of *embrio-tcp*))))))

(defun embrio-gripper-move (args)
  (let* ((width (embrio-number (embrio-arg args :position) "position"))
         (speed (embrio-positive (embrio-arg args :speed) "speed"))
         (force (embrio-positive (embrio-arg args :force) "force"))
         (width-m (/ width 1000.0))
         (speed-m (/ speed 1000.0)))
    (unless (<= 0.0 width-m *embrio-gripper-open-max-m*)
      (error "position must be between 0 and ~a mm" (* 1000.0 *embrio-gripper-open-max-m*)))
    (embrio-sync-model)
    ;; Closing goes through the grasp action so jaws stopped by an object still hold with `force`.
    (if (< width (embrio-gripper-mm))
        (send *ri* :send-gripper-grasp-action :rarm width-m speed-m force :inner width-m :outer *embrio-gripper-open-max-m*)
      (send *ri* :send-gripper-move-action :rarm width-m speed-m))
    (embrio-sync-model)
    (list (cons "achieved" (embrio-gripper-mm)))))

(defun embrio-gripper-position (args)
  (embrio-sync-model)
  (list (cons "position" (embrio-gripper-mm))))

(defun embrio-run-measurement (args)
  (let ((thickness (embrio-positive (embrio-arg args :thickness) "thickness"))
        (sample-name (embrio-arg args :sample_name))
        (file-path (embrio-arg args :file_path))
        (do-measure (embrio-arg args :do_measure))
        (insert-until-contact (embrio-arg args :insert_until_contact))
        (measure-times (embrio-arg args :measure_times)))
    (unless (and (stringp sample-name) (> (length sample-name) 0)) (error "sample_name must be a non-empty string"))
    (unless (and (stringp file-path) (> (length file-path) 0)) (error "file_path must be a non-empty string"))
    (unless (and (integerp measure-times) (>= measure-times 1)) (error "measure_times must be an integer >= 1"))
    (if (= measure-times 1)
        (pick-polymer-and-measure-40-28-10
         :thickness thickness :sample-name sample-name :file-path file-path
         :do-measure do-measure :insert-until-contact insert-until-contact)
      (pick-polymer-and-measure-40-28-10-ntimes-each
       :measure-times measure-times
       :thickness thickness :sample-name sample-name :file-path file-path
       :do-measure do-measure :insert-until-contact insert-until-contact))
    (list (cons "measure_times" measure-times))))

(defvar *embrio-handlers*
  (list (cons "ping" 'embrio-ping)
        (cons "pose" 'embrio-pose)
        (cons "move_joints" 'embrio-move-joints)
        (cons "move_position" 'embrio-move-position)
        (cons "gripper_move" 'embrio-gripper-move)
        (cons "gripper_position" 'embrio-gripper-position)
        (cons "run_measurement" 'embrio-run-measurement)))

(defun embrio-dispatch (text)
  (let ((outcome
         (catch :embrio-abort
           (catch :embrio-error
             (let* ((*embrio-in-call* t)
                    (form (embrio-parse text))
                    (name (string-downcase (symbol-name (car form))))
                    (handler (cdr (assoc name *embrio-handlers* :test #'string=))))
               (unless handler (error "unknown method ~a" name))
               (setq *embrio-busy* t)
               (unwind-protect
                   (list :ok (funcall handler (cdr form)))
                 (setq *embrio-busy* nil)))))))
    (setq *embrio-busy* nil)
    (case (car outcome)
      (:ok (embrio-json (list (cons "ok" t) (cons "result" (cadr outcome)))))
      (:aborted (embrio-json (list (cons "ok" :false) (cons "error" "aborted"))))
      (t (embrio-json (list (cons "ok" :false) (cons "error" (cadr outcome))))))))

(defun embrio-call-cb (req)
  (let ((res (send req :response)))
    (send res :str (embrio-dispatch (send req :str)))
    res))

;; Reached from ros::spin-once inside a running handler's :wait-interpolation, which is the
;; only place the node listens while it moves.
(defun embrio-abort-cb (msg)
  (send *ri* :cancel-angle-vector)
  (when *embrio-busy*
    (throw :embrio-abort (list :aborted))))

(defun main ()
  (ros::roseus "auto_dem_embrio_manip")
  (ros::rate 10)
  (ros::advertise-service "/embrio/auto_dem/call" roseus::StringString #'embrio-call-cb)
  (ros::subscribe "/embrio/auto_dem/abort" std_msgs::Empty #'embrio-abort-cb)
  (objects *display-objects*)
  (warn "auto_dem_embrio_manip: waiting for /embrio/auto_dem/call~%")
  (do-until-key
   (ros::spin-once)
   (x::window-main-one)
   (ros::sleep)))

(main)
