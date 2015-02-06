(in-package :planlib)


(if (not (boundp '+scan-poses+))
(defconstant +scan-poses+ (list "shadow_pose1" "shadow_pose2") "Poses that can't be scanned by the mast cam"))

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

(def-cram-function scan-shadow ()
  (perform (make-designator 'action `((to move-arm-cam) (pose-name ,(first +scan-poses+)))))
  (perform (make-designator 'action `((to perceive-scene-with) (scenecam nil) (arm-origin ,(value manipulation:*base-origin*)))))
  (perform (make-designator 'action `((to move-arm-cam ) (pose-name ,(second +scan-poses+)))))
  (perform (make-designator 'action `((to perceive-scene-with) (scenecam nil) (arm-origin ,(value manipulation:*base-origin*))))))

(def-goal (cram-plan-library:achieve (map-scanned))
  (scan-map-mast-cam)
  (scan-shadow))
