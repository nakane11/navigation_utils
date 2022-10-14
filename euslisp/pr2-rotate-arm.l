#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::roseus-add-srvs "std_srvs")

(defun pr2-rotate-arm ()
  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (+ (elt default 1) 40)) :start1 1)
  (setq ac (pr2-interface-move-base-trajectory-action *ri*))
  (setq goal (send *ri* :move-trajectory-sequence
                   (list #f(0 0 (deg2rad 25))
                         #f(0 0 (deg2rad -25))
                         #f(0 0 (deg2rad -175))
                         #f(0 0 (deg2rad 135)))
                   (list 800 800 2500 2000)))
  (send *ri* :angle-vector-sequence default 800 :arm-controller)
  (send ac :send-goal goal)
  (send *ri* :wait-interpolation))

(defun callback (req)
    (pr2-rotate-arm)
    (instance std_srvs::EmptyResponse :init))

(ros::roseus "partial_rotate")
(pr2-init)
(ros::advertise-service "~partial_rotate" std_srvs::Empty #'callback)
(do-until-key
 (ros::spin-once))
