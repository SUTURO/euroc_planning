(in-package :planlib)

(def-goal (achieve (map-scanned))
  (scan-map-mast-cam)
  (scan-shadow))

(def-goal (achieve (objects-informed))
  ; TODO: Implement me correctly
  (achieve `(unknown-scanned))
  (achieve `(object-classified nil))
  (achieve `(pose-estimated nil)))

(def-goal (achieve (unknown-scanned))
  ; TODO: Implement me correcty
  (perform (make-designator 'action '((to move-arm-cam)
                                      (pose-name nil))))
  (perform (make-designator 'action '((to perceive-scene-with)
                                      (scenecam nil)))))

(def-goal (achieve (object-classified ?object))
  ; TODO: Implement me correcty
  (perform (make-designator 'action `((to classify-object)
                                      (obj ,?object)))))

(def-goal (achieve (pose-estimated ?object))
    (pose-estimate-object ?object)
  (perform (make-designator 'action `((to focus-object)
                                      (obj ,?object))))
  (perform (make-designator 'action `((to pose-estimate-object)
                                      (obj ,?object)))))

(def-goal (achieve (objects-in-place ?objects))
  (let ((target-zones (get-target-zones)))
    (mapcar (lambda (object)
              (let ((matching-target-zone (find-matching-target-zone object target-zones)))
                (with-retry-counters ((pick-up-retry-count 2)
                                      (put-down-retry-count 2))
                  (seq
                    (with-failure-handling
                      (((or manipulation-failure
                            manipulation-pose-unreachable) (e)
                        (declare (ignore e))
                        (ros-warn (objects-in-place) "Failed to pick up object.")
                        (do-retry pick-up-retry-count
                          (ros-warn (objects-in-place) "Retrying.")
                          (retry))))
                      (achieve `(object-in-hand ,object))
                      (equate (current-desig object)
                              (copy-designator (current-desig object)
                                               :new-description `((at ,(make-designator 'location '((gripper gripper))))))))
                    (with-failure-handling
                      (((or manipulation-failure
                            manipulation-pose-unreachable) (e)
                        (declare (ignore e))
                        (ros-warn (objects-in-place) "Failed to put down object.")
                        (do-retry put-down-retry-count
                          (ros-warn (objects-in-place) "Retrying.")
                          (retry))))
                      (achieve `(object-placed-at ,(current-desig object) ,matching-target-zone))
                      (equate (current-desig object)
                              (copy-designator (current-desig object)
                                               :new-description `((at ,matching-target-zone)))))))))
            ?objects)))
