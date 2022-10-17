#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(ros::roseus-add-srvs "std_srvs")

(defun fetch-rotate-arm ()
  (setq default (send *ri* :state :potentio-vector))
  (replace default (list 50) :start1 1)
  (send *ri* :angle-vector-raw default 1000 :arm-controller)
  (send *ri* :go-velocity 0 0 (/ (deg2rad -50) 1) 1000)
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list -90) :start1 1)
  (send *ri* :angle-vector-raw default (* 1000 2.8) :arm-controller)
  (send *ri* :go-velocity 0 0 (/ (deg2rad 140) (* 1 2.8)) (* 1000 2.8))
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list 0) :start1 1)
  (send *ri* :angle-vector-raw default (* 1000 1.4) :arm-controller)
  (send *ri* :go-velocity 0 0 (/ (deg2rad -90) (* 1 1.4)) (* 1000 1.4))
  (send *ri* :wait-interpolation))

(defun callback (req)
    (fetch-rotate-arm)
    (instance std_srvs::EmptyResponse :init))

(ros::roseus "partial_rotate")
(fetch-init)
(ros::advertise-service "~partial_rotate" std_srvs::Empty #'callback)
(do-until-key
 (ros::spin-once))

;; (send *ri* :angle-vector-raw #f(0.0 0.0 90 90 120 0 60 90 0 0) 800 :arm-controller)
