(in-package :planlib)

(def-goal (achieve (map-scanned))
  (scan-map-mast-cam)
  (scan-shadow))

(def-goal (achieve (objects-informed))
  (with-retry-counters ((unknown-scanned-retry-count 2))
    (with-failure-handling
      ((simple-plan-failure (e)
         (declare (ignore e))
         (ros-warn (objects-informed) "Failed to scan unknown regions.")
         (do-retry unknown-scanned-retry-count
           (ros-warn (objects-informed) "Retrying.")
           (retry))
         (fail 'objects-information-failed)))
      (let ((regions (achieve `(objects-located ,(cl-utilities:copy-array(roslisp:msg-slot-value exec:*yaml* 'objects))))))
        (loop for region across regions do
                  ;;TODO achieve objects-located in Fehlerbehandlung einbauen
                  (let ((objects-in-scene (achieve `(unknown-scanned ,region))))
                    (loop for object across objects-in-scene do
                      (with-retry-counters ((object-classified-retry-count 2)
                                            (pose-estimated-retry-count 2))
                        (with-failure-handling
                            ((simple-plan-failure (e)
                                                  (declare (ignore e))
                                                  (ros-warn (objects-informed) "Failed to classify object.")
                                                  (do-retry object-classified-retry-count
                                                    (ros-warn (objects-informed) "Retrying.")
                                                    (retry))
                                                  (fail 'objects-information-failed)))
                          (let ((classified-object (achieve `(object-classified object))))
                            (with-failure-handling
                                ((simple-plan-failure (e)
                                                      (declare (ignore e))
                                                      (ros-warn (objects-informed) "Failed to estimate pose for object.")
                                                      (do-retry pose-estimated-retry-count
                                                        (ros-warn (objects-informed) "Retrying.")
                                                        (retry))
                                                      (fail 'objects-information-failed)))
                              (achieve `(pose-estimated classified-object)))))))))))))

(def-goal (achieve (objects-located ?objects))
  (perform (make-designator 'action `((to find-objects-in-map) 
                                      (objects ,?objects)))))

(def-goal (achieve (unknown-scanned ?region))
  (look-at-obstacle ?region)
  (roslisp:msg-slot-value (perform (make-designator 'action `((to get-gripper-perception)))) 'objects))

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
