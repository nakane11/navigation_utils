#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
;; (ros::roseus "rotate-arm")
;; (fetch-init)

(defun fetch-rotate-arm (&optional (range (/ pi 3)) (time 1000))

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (+ (elt default 3) range)) :start1 3)
  (send *ri* :angle-vector default time :arm-controller)
  (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  (setq goal (send *ri* :move-trajectory 0 0 range time)) 
  (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (- (elt default 3) range)) :start1 3)
  (send *ri* :angle-vector default time :arm-controller)
  (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  (setq goal (send *ri* :move-trajectory 0 0 -range time)) 
  (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (- (elt default 3) range)) :start1 3)
  (send *ri* :angle-vector default time :arm-controller)
  (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  (setq goal (send *ri* :move-trajectory 0 0 -range time)) 
  (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation)

  (setq default (send *ri* :state :potentio-vector))
  (replace default (list (+ (elt default 3) range)) :start1 3)
  (send *ri* :angle-vector default time :arm-controller)
  (setq ac (fetch-interface-move-base-trajectory-action *ri*))
  (setq goal (send *ri* :move-trajectory 0 0 range time)) 
  (send ac :send-goal goal) ;; robot will start to move
  (send *ri* :wait-interpolation))

;; (send *ri* :go-velocity 0 0 (* (/ ((/ range 180) pi) time) 1000) time)


