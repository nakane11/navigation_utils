#!/usr/bin/env roseus

(ros::roseus-add-msgs "nav_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-srvs "std_srvs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "pointing")

(setq *look-enable* t)
(setq *speak-timestamp* (ros::time 0))

(setq *rarm-point-l1* #f(32 22 -13 -95 -157 -64 10))
(setq *rarm-point-l2* #f(-10 46 -56 -122 -132 -50 10))
(setq *rarm-point-l3* #f(-17 66 -106 -97 9 -6 10))
(setq *rarm-tuck* #f(-5 70 -105 -90 70 -6 10))

(setq *rarm-point-r1* #f(-18 31 10 -118 -214 -78 10))
(setq *rarm-point-r2* #f(6 48 -26 -121 -146 -72 10))
(setq *rarm-point-r3* #f(-17 66 -106 -97 9 -6 10))

(defun get-angle-vector (arg arm)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* arm :angle-vector arg)
  (send *pr2* :angle-vector))

(defun rarm-point (dir)
  (when (eq dir :left)
    (setq av1 (get-angle-vector *rarm-point-l1* :rarm))
    (setq av2 (get-angle-vector *rarm-point-l2* :rarm))
    (setq av3 (get-angle-vector *rarm-point-l3* :rarm))
    (setq avt (get-angle-vector *rarm-tuck* :rarm))
    (send *ri* :angle-vector-sequence (list av3 av2 av1 av2 av3 avt) (list 300 300 300 400 400 400) :rarm-controller))
  (when (eq dir :right)
    (unix:sleep 5)
    (setq av1 (get-angle-vector *rarm-point-r1* :rarm))
    (setq av2 (get-angle-vector *rarm-point-r2* :rarm))
    (setq av3 (get-angle-vector *rarm-point-r3* :rarm))
    (setq avt (get-angle-vector *rarm-tuck* :rarm))
    (send *ri* :angle-vector-sequence (list av3 av2 av1 av2 av3 avt) (list 300 300 300 400 400 400) :rarm-controller)))  

;; if (ros::spin-once) is called in callback function,
;;   stack overflow will occure.
(defun global-path-cb (msg)
  (setq *msg* msg))

(defun result-cb (msg)
  (let ((av (send *ri* :state :potentio-vector)))
    (unless *look-enable*
      (return-from result-cb nil))
    (send *pr2* :angle-vector av)
    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
    ))

(defun look-at-front ()
  (unless (boundp '*msg*) (return-from look-at-front nil))
  (if (< 5 (send (ros::time- (ros::time-now)
			     (send *msg* :header :stamp)) :to-sec))
      (return-from look-at-front nil))
  ;;
  (let* ((po (instance geometry_msgs::PoseStamped :init
		       :header (send *msg* :header)
		       :pose (send (car (last (send *msg* :poses))) :pose)))
	 tra (av (send *pr2* :angle-vector)))
    ;; need convert ?
    (send *tfl* :wait-for-transform
	  "base_footprint" (send po :header :frame_id)
	  (send po :header :stamp) 1)
    (setq tra (send *tfl* :transform-pose "base_footprint" po))
    (when (not tra)
      (ros::ros-error "not transform")
      (return-from look-at-front nil))
    (ros::ros-info "msg received ~A~%" *msg* (norm (send tra :worldpos)))
    (ros::ros-info "look-forward enabled: ~A" *look-enable*)

    (when *look-enable*
      (if (< 500 (norm (send tra :worldpos)))
          (send *pr2* :head :look-at (v+ #f(0 0 1300) (send tra :worldpos)))
          (send *pr2* :head :look-at #f(1000 0 1300)))
      (if (> (abs (send *pr2* :head :neck-p :joint-angle)) 30)
          (ros::ros-error "error head ik ~A ~A" av (v+ #f(0 0 1300) (send tra :worldpos)))
          (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)))

    (when (and (< 3.0 (send (ros::time- (ros::time-now) *speak-timestamp*) :to-sec))
	       (< 1000 (norm (send tra :worldpos))))
      (setq ang (atan (elt (send tra :worldpos) 1)
		      (elt (send tra :worldpos) 0)))
      (when (< (* 0.25 pi) ang (* 0.50 pi))
    (rarm-point :left)
	(speak-jp "ひだりに/まがり'ます。")
	(setq *speak-timestamp* (ros::time-now)))
      (when (> (* -0.25 pi) ang (* -0.50 pi))
    (rarm-point :right)
	(speak-jp "みぎに/まがり'ます。")
	(setq *speak-timestamp* (ros::time-now)))
      (ros::ros-info "angle = ~A" ang)
      )
    (ros::ros-info "~A plan trajectory end point ~A, head angle ~A" *msg* (send tra :worldpos) (send *pr2* :head :angle-vector))
    ))

(defun start-look-front (req)
  (setq *look-enable* t)
  (instance std_srvs::EmptyResponse :init))

(defun stop-look-front (req)
  (setq *look-enable* nil)
  (instance std_srvs::EmptyResponse :init))

;; init
(pr2-init)
(defparameter *tfl* (instance ros::transform-listener :init))
(ros::subscribe "/move_base_node/DWAPlannerROS/global_plan"
                nav_msgs::Path #'global-path-cb 1)
;; look straight when navigation retries
(ros::subscribe "/move_base/result"
                move_base_msgs::MoveBaseActionResult #'result-cb 1)

(ros::advertise-service "~start" std_srvs::Empty #'start-look-front)
(ros::advertise-service "~stop" std_srvs::Empty #'stop-look-front)

(unix::sleep 1)

(ros::spin-once)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))

;;
(while (ros::ok)
  (ros::rate 1)
  (ros::sleep)
  (ros::spin-once)
  (look-at-front))
