(in-package :planlib)

(defstruct (mast-cam-position :conc-name)
  "Represents the position of the mast cam"
  (pan 0)
  (tilt 0))

(defconstant +mast-cam-position-1+ (make-mast-cam-position :pan 0.2 :tilt 0.5))
(defconstant +mast-cam-position-2+ (make-mast-cam-position :pan 0.2825 :tilt 0.775))
(defconstant +mast-cam-position-3+ (make-mast-cam-position :pan 0 :tilt 1.1))
(defconstant +mast-cam-position-4+ (make-mast-cam-position :pan -0.2825 :tilt 0.775))
(defconstant +mast-cam-position-5+ (make-mast-cam-position :pan -0.2 :tilt 0.5))

(defun scan-map-part(pan tilt)
  (if (not (roslisp:wait-for-service +service-name-move-mastcam+ *timeout-service*))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name+))
      (progn
        (roslisp:call-service +service-name-move-mastcam+ 'suturo_planning_manipulation-srv:MoveMastCam :pan pan :tilt tilt)
        (sleep +waiting-time-before-scan+)
        (if (not (roslisp:wait-for-service +service-name-add-point-cloud+ *timeout-service*))
            (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-add-point-cloud+))
            (roslisp:call-service +service-name-add-point-cloud+ 'suturo_interface_msgs-srv:AddPointCloud :scenecam T)))))

(defun scan-map ()
  (scan-map-part (pan +mast-cam-position-1+) (tilt +mast-cam-position-1+))
  (scan-map-part (pan +mast-cam-position-2+) (tilt +mast-cam-position-2+))
  (scan-map-part (pan +mast-cam-position-3+) (tilt +mast-cam-position-3+))
  (scan-map-part (pan +mast-cam-position-4+) (tilt +mast-cam-position-4+))
  (scan-map-part (pan +mast-cam-position-5+) (tilt +mast-cam-position-5+)))

;(defun init ()
;  (format t "starting")
;  (whenever ((pulsed (fl-or *current-state* *current-transition*)))
;    (when (and (eql (value *current-state*) :state-init) (eql (value *current-transition*) :state-successful))
;      (format t "done"))))

(def-cram-function state-scan-map ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-init) (eql *current-transition* :transition-successful)))
      (print "Executing state scan map")
      (setf (value *current-state*) :state-scan-map)
      (scan-map)
      (setf (value *current-transition*) :transition-map-scanned)))

;(cpl-impl:wait-for (fl-and (eql *our-fluent* :state-init) (eql *our-fluent2* :transition-successful)))
;(cpl-impl:top-level
;           (cpl-impl:pursue
;             (cpl-impl:whenever ((cpl:pulsed *our-fluent*))
;               (let ((value (cpl:value *our-fluent*)))
;                 (when (and (stringp value) (string= "huhu" value))
;                   (format t "Huhu they said.~%"))
;                 (format t "FLUENT: ~a~%" value)))
;             (progn
;               (cpl:sleep 1)
;               (setf (cpl:value *our-fluent*) 8)
;               (cpl:sleep 1))))


