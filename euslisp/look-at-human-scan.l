#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")

(defvar *look-at-human-enabled* nil)

(defun pose-array-cb (robot msg)
  (let ((min-pos nil) (min-d 2000.0) (tm (instance ros::Time :init)))
    (when (ros::time> *next-look-time* (ros::time-now))
      (return-from pose-array-cb nil))
    (unless (> (length (send msg :poses)) 0)
      (return-from pose-array-cb nil))

    (dotimes (i (length (send msg :poses)))
      (let* ((person (elt (send msg :poses) i))
             (person-coords (ros::tf-pose->coords person)))
        (setq stamp (send msg :header :stamp))
        (setq base-to-map (send *tfl* :lookup-transform "/base_link" "/map" stamp))
        (when base-to-map
          (setq person-pos (send (send base-to-map :transform person-coords) :pos))
          (setq d (norm (float-vector (elt person-pos 0) (elt person-pos 1))))
          (when (> (elt person-pos 1) 0)
            (when (< d min-d)
              (setq min-pos (float-vector (elt person-pos 0) (elt person-pos 1) 1100))
              (setq min-d d)))
          )))
    (ros::ros-info (format nil "d:~A" min-d))

    ;; Do nothing when no valid limb was found
    (when (null min-pos)
      (return-from pose-array-cb nil))

    ;; Look at target
    (look-at-target robot min-pos)
    (setq interval (+ 10 (random 20)))
    (ros::ros-info (format nil "next time ~A" interval))
    (send tm :from-sec interval)
    (setq *next-look-time* (ros::time+ (ros::time-now) tm))
    ))

(defun look-at-target (robot look-xyz)
  (send robot :angle-vector (send *ri* :state :potentio-vector))
  (send robot :head :look-at look-xyz)
  (send *ri* :angle-vector (send robot :angle-vector) (+ 900 (random 400)) :head-controller)
  (send *ri* :wait-interpolation)
  (send robot :angle-vector (send *ri* :state :potentio-vector))
  (send robot :head :angle-vector #f(0.0 0.0))
  (send *ri* :angle-vector (send robot :angle-vector) (+ 800 (random 100)) :head-controller)
)

(defun start-look-at-human (robot req)
  (ros::ros-info "Start looking at human.")
  (setq *look-at-human-enabled* t)
  (ros::subscribe
   "/dr_spaam_lidar_person_detection/output" geometry_msgs::PoseArray
   #'pose-array-cb robot)
  (instance std_srvs::EmptyResponse :init))

(defun stop-look-at-human (req)
  (ros::ros-info "Stop looking at human.")
  (setq *look-at-human-enabled* nil)
  (ros::unsubscribe "/dr_spaam_lidar_person_detection/output")
  (instance std_srvs::EmptyResponse :init))

(defun main (robot)
  (ros::advertise-service "~start" std_srvs::Empty #'start-look-at-human robot)
  (ros::advertise-service "~stop" std_srvs::Empty #'stop-look-at-human)
  (ros::advertise "~enabled" std_msgs::Bool 1)
  (unix:usleep (* 100 1000))

  (ros::rate 2)
  (while (ros::ok)
    (ros::spin-once)
    (ros::publish "~enabled"
                  (instance std_msgs::Bool :init :data *look-at-human-enabled*))
    ))

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::roseus "look_at_human_scan")
(defvar *next-look-time* (ros::time-now))
(pr2-init)
(main *pr2*)
