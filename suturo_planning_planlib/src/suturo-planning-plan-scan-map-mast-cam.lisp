(in-package :planlib)

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
  
)

(def-goal (cram-plan-library:achieve (map-scanned))
    (scan-map-mast-cam))
