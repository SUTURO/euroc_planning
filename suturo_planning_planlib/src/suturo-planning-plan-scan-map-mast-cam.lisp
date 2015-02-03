(in-package :planlib)

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


(def-cram-function scan-map-mast-cam ()
  "Scans the map"
  (perform (make-designator 'action '((to move-mast-cam) (pan 0.2) (tilt 0.5))))
  (perform (make-designator 'action '((to perceive-scene-with) (scenecam T))))
  (perform (make-designator 'action '((to move-mast-cam) (pan 0.2825) (tilt 0.775))))
  (perform (make-designator 'action '((to perceive-scene-with) (scenecam T))))
  (perform (make-designator 'action '((to move-mast-cam) (pan 0) (tilt 1.1))))
  (perform (make-designator 'action '((to perceive-scene-with) (scenecam T))))
  (perform (make-designator 'action '((to move-mast-cam) (pan -0.2825) (tilt 0.775))))
  (perform (make-designator 'action '((to perceive-scene-with) (scenecam T))))
  (perform (make-designator 'action '((to move-mast-cam) (pan -0.2) (tilt 0.5))))
  (perform (make-designator 'action '((to perceive-scene-with) (scenecam T)))))

(defun asdf ()
  (make-designator 'action '((to perceive-scene-with) (scenecam T))))


(def-goal (cram-plan-library:achieve (map-scanned))
    (scan-map-mast-cam))
 
