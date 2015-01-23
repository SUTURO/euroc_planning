(in-package :manipulation)

(def-action-handler scan-map ()
  "Scans the map"
  (scan-map)
  (scan-poses)
)

(defstruct (mast-cam-position :conc-name)
  "Represents the position of the mast cam"
  (pan 0)
  (tilt 0))

;;The following constants describe the positions of the map scan, that should be scanned 
(defconstant +mast-cam-position-1+ (make-mast-cam-position :pan 0.2 :tilt 0.5))
(defconstant +mast-cam-position-2+ (make-mast-cam-position :pan 0.2825 :tilt 0.775))
(defconstant +mast-cam-position-3+ (make-mast-cam-position :pan 0 :tilt 1.1))
(defconstant +mast-cam-position-4+ (make-mast-cam-position :pan -0.2825 :tilt 0.775))
(defconstant +mast-cam-position-5+ (make-mast-cam-position :pan -0.2 :tilt 0.5))

(defconstant +move-group-arm+ 0 "Type parameter for the service call +service-name-move-robot+")
(defconstant +move-group-arm-base+ 1 "Type parameter for the service call +service-name-move-robot+")
(defconstant MOVE_GROUP_BASE 2 "Type parameter for the service call +service-name-move-robot+")
(defconstant +scan-poses+ (list "shadow_pose1" "shadow_pose2") "Poses that can't be scanned by the mast cam")


(defun scan-map-part(pan tilt)
  "Moves the mast cam to the given position and adds the scanned point cloud to the current scene."
  (if (not (roslisp:wait-for-service +service-name-move-mastcam+ *timeout-service*))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name+))
      (progn
        (roslisp:call-service +service-name-move-mastcam+ 'suturo_planning_manipulation-srv:MoveMastCam :pan pan :tilt tilt)
        (sleep +waiting-time-before-scan+)
        (if (not (roslisp:wait-for-service +service-name-add-point-cloud+ *timeout-service*))
            (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-add-point-cloud+))
            (roslisp:call-service +service-name-add-point-cloud+ 'suturo_interface_msgs-srv:AddPointCloud :scenecam T)))))


(defun scan-map ()
  "Scans the map"
  (scan-map-part (pan +mast-cam-position-1+) (tilt +mast-cam-position-1+))
  (scan-map-part (pan +mast-cam-position-2+) (tilt +mast-cam-position-2+))
  (scan-map-part (pan +mast-cam-position-3+) (tilt +mast-cam-position-3+))
  (scan-map-part (pan +mast-cam-position-4+) (tilt +mast-cam-position-4+))
  (scan-map-part (pan +mast-cam-position-5+) (tilt +mast-cam-position-5+)))


(defun scan-poses () 
  (print "scanning poses")
  (loop for pose in +scan-poses+ do 
    (print (concatenate 'string "Taking pose: " pose))
    ;Move the robot to one of the shadow position
    (if (not (roslisp:wait-for-service +service-name-move-robot+ *timeout-service*))
        (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-move-robot+))
        (progn
          (roslisp:call-service +service-name-move-robot+ 'suturo_planning_manipulation-srv:Move :type +move-group-arm+ :goal_pose_name pose)
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
