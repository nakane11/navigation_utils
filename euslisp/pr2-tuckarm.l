#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2-tuckarm")
(pr2-init)
(pr2-tuckarm-pose)
