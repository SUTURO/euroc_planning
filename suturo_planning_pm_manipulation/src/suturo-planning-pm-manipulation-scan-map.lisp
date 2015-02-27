(in-package :manipulation)

(def-action-handler move-mast-cam (pan tilt) 
  "Moves the mast cam to the given position"
  (if (not (roslisp:wait-for-service +service-name-move-mastcam+ +timeout-service+))
      (progn
        (roslisp:ros-warn (manipulation move-mast-cam) t (concatenate 'string "Following service timed out: " +service-name-move-mastcam+))
        (cpl-impl:fail 'moving-mast-cam-failed))
      (roslisp:call-service +service-name-move-mastcam+ 'suturo_manipulation_msgs-srv:MoveMastCam :pan pan :tilt tilt)))

(def-action-handler move-arm-cam-pose-name (pose) 
  "Moves the mast cam to the given position"
  (if (not (roslisp:wait-for-service +service-name-move-robot+ +timeout-service+))
      (progn
        (roslisp:ros-warn (manipulation move-arm-cam-pose-name) t (concatenate 'string "Following service timed out: " +service-name-move-robot+))
        (cpl-impl:fail 'moving-arm-failed))
      (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move 
                            :type  (roslisp:symbol-code (roslisp-msg-protocol:service-request-type 'suturo_manipulation_msgs-srv:Move) :ACTION_MOVE_ARM_TO)
                            :goal_pose_name pose)))

