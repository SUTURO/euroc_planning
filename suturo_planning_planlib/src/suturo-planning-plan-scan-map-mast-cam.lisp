(in-package :planlib)


(if (not (boundp '+scan-poses+))
(defconstant +scan-poses+ (list "shadow_pose1" "shadow_pose2") "Poses that can't be scanned by the mast cam"))

(def-cram-function scan-map-mast-cam ()
  "Scans the map"
  (let ((pans '(0.2 0.2825 0 -0.2825 -0.2))
        (tilts '(0.5 0.775 1.1 0.775 0.5)))
    (mapcar (lambda (pan tilt)
              (with-retry-counters ((move-mast-cam-retry-count 2)
                                    (perceive-scene-retry-count 2))
                (with-designators
                  ((move-mast-cam (action `((to move-mast-cam) (pan ,pan) (tilt ,tilt))))
                   (perceive-scene (action '((to perceive-scene-with) (scenecam t)))))
                  (with-failure-handling
                    ((moving-mast-cam-failed (e)
                       (declare (ignore e))
                       (ros-warn (scan-map-mast-cam) "Failed to move mast cam.")
                       (do-retry move-mast-cam-retry-count
                         (ros-warn (scan-map-mast-cam) "Retrying.")
                         (retry))))
                    (perform move-mast-cam)
                    (with-failure-handling
                      ((map-scanning-failed (e)
                         (declare (ignore e))
                         (ros-warn (scan-map-mast-cam) "Failed to scan map.")
                         (do-retry perceive-scene-retry-count
                           (ros-warn (scan-map-mast-cam) "Retrying.")
                           (retry))))
                      (perform perceive-scene))))))
            pans tilts)))

(def-cram-function scan-shadow ()
  (perform (make-designator 'action `((to move-arm-cam) (pose-name ,(first +scan-poses+)))))
  (perform (make-designator 'action `((to perceive-scene-with) (scenecam nil) (arm-origin ,(value manipulation:*base-origin*)))))
  (perform (make-designator 'action `((to move-arm-cam ) (pose-name ,(second +scan-poses+)))))
  (perform (make-designator 'action `((to perceive-scene-with) (scenecam nil) (arm-origin ,(value manipulation:*base-origin*))))))
