(in-package :planlib)

(defconstant +move-group-arm+ 0 "Type parameter for the service call +service-name-move-robot+")
(defconstant +move-group-arm-base+ 1 "Type parameter for the service call +service-name-move-robot+")
(defconstant MOVE_GROUP_BASE 2 "Type parameter for the service call +service-name-move-robot+")
(defconstant +scan-poses+ (list "shadow_pose1" "shadow_pose2") "Poses that can't be scanned by the mast cam")

(defun scan-poses () 
  (print "scanning poses")
  (loop for pose in +scan-poses+ do 
    (print (concatenate 'string "Taking pose: " pose))
    ;Move the robot to one of the shadow position
    (if (not (roslisp:wait-for-service +service-name-move-robot+ *timeout-service*))
        (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-move-robot+))
        (progn
          (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move :type +move-group-arm+ :goal_pose_name pose)
          (sleep +waiting-time-before-scan+)
          ;Get the base origin 
          (if (not (roslisp:wait-for-service +service-name-get-base-origin+ *timeout-service*)) 
              (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-move-robot+))
              (let
                  ((msg (roslisp:call-service +service-name-get-base-origin+ 'suturo_interface_msgs-srv:GetBaseOrigin)))
                ;Add the point cloud to the map
                (if (not (roslisp:wait-for-service +service-name-add-point-cloud+ *timeout-service*))
                    (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-add-point-cloud+))
                    (roslisp:call-service +service-name-add-point-cloud+ 'suturo_interface_msgs-srv:AddPointCloud :arm_origin (roslisp:msg-slot-value msg 'base_origin) :scenecam nil))))))))

(def-cram-function state-scan-shadow ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-scan-map) (eql *current-transition* :transition-map-scanned)))
      (print "Executing state scan shadow")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-scan-shadow)
      (scan-poses)
      (setf (value *current-transition*) :transition-success)))
