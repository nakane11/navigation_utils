#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(ros::roseus-add-srvs "std_srvs")

(defun fetch-rotate-arm ()
  (send *ri* :angle-vector-sequence-raw (list #f(0 50 90 90 120 0 60 90 0 0)
                                              #f(0 -90 90 90 120 0 60 90 0 0)
                                              #f(0 0 90 90 120 0 60 90 0 0))
        (list 2500 (* 2500 2.8) (* 2500 1.4))
        :arm-controller)
  (send *ri* :move-trajectory-sequence (list (float-vector 0 0 (deg2rad -50))
                                             (float-vector 0 0 (deg2rad 90))
                                             (float-vector 0 0 (deg2rad -90)))
        (list 2500 (* 2500 2.8) (* 2500 1.4))
        :send-action t)
  (send *ri* :wait-interpolation))

(defun callback (req)
    (fetch-rotate-arm)
    (instance std_srvs::EmptyResponse :init))

(ros::roseus "partial_rotate")
(fetch-init)
(ros::advertise-service "~partial_rotate" std_srvs::Empty #'callback)
(do-until-key
 (ros::spin-once))
