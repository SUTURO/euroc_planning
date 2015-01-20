(in-package :planlib)

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


(def-cram-function state-scan-map ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-init) (eql *current-transition* :transition-successful)))
      (print "Executing state scan map")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-scan-map)
      (scan-map)
      (setf (value *current-transition*) :transition-map-scanned)))
