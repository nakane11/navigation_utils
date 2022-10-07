#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(ros::roseus-add-srvs "std_srvs")

(defun fetch-rotate-arm ()
  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (+ (elt default 1) 40)) :start1 1)
  (send *ri* :angle-vector-raw default 800 :arm-controller)
  (send *ri* :go-velocity 0 0 (/ (deg2rad -40) 0.6) 600)
  ;; (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  ;; (setq goal (send *ri* :move-trajectory 0 0 (deg2rad 25) 800))
  ;; (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (- (elt default 1) 175)) :start1 1)
  (send *ri* :angle-vector-raw default 2500 :arm-controller)
  (send *ri* :go-velocity 0 0 (/ (deg2rad 175) 2.4) 2400)
  ;; (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  ;; (setq goal (send *ri* :move-trajectory 0 0 -(deg2rad 25) 800))
  ;; (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (+ (elt default 1) 135)) :start1 1)
  (send *ri* :angle-vector-raw default 2000 :arm-controller)
  (send *ri* :go-velocity 0 0 (/ (deg2rad -135) 2) 2000)
  ;; (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  ;; (setq goal (send *ri* :move-trajectory 0 0 (deg2rad -135) 2000))
  ;; (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation))

(defun callback (req)
    (fetch-rotate-arm)
    (instance std_srvs::EmptyResponse :init))

(ros::roseus "partial_rotate")
(fetch-init)
(ros::advertise-service "~partial_rotate" std_srvs::Empty #'callback)
(do-until-key
 (ros::spin-once))
