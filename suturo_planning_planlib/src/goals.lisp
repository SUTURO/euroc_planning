(in-package :planlib)

(def-goal (achieve (map-scanned))
  (scan-map-mast-cam)
  (scan-shadow))

(def-goal (achieve (objects-informed))
  (let ((resp (list)))
    (with-retry-counters ((objects-located-retry-count 2))
      (with-failure-handling
        ((simple-plan-failure (e)
           (declare (ignore e))
           (ros-warn (objects-informed) "Failed to locate objects.")
           (do-retry objects-located-retry-count
             (ros-warn (objects-informed) "Retrying.")
             (retry))
           (fail 'objects-information-failed)))
             (let ((regions (achieve `(objects-located ,(if environment:*yaml*
                                                          (cl-utilities:copy-array(roslisp:msg-slot-value environment:*yaml* 'objects))
                                                          (progn
                                                            (ros-error (objects-informed) "environment:*yaml* not set.")
                                                            (fail 'objects-information-failed)))))))
                 (loop for region across regions do
                   (with-retry-counters ((unknown-scanned-retry-count 2))
                     (with-failure-handling
                       ((simple-plan-failure (e)
                          (declare (ignore e))
                          (ros-warn (objects-informed) "Failed to scan unknown regions.")
                          (do-retry unknown-scanned-retry-count
                            (ros-warn (objects-informed) "Retrying.")
                            (retry))
                          (fail 'objects-information-failed)))
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
                               (let ((classified-object (achieve `(object-classified ,object))))
                                 (with-failure-handling
                                   ((simple-plan-failure (e)
                                      (declare (ignore e))
                                      (ros-warn (objects-informed) "Failed to estimate pose for object.")
                                      (do-retry pose-estimated-retry-count
                                        (ros-warn (objects-informed) "Retrying.")
                                        (retry))
                                      (fail 'objects-information-failed)))
                                 (setq resp (append resp (list (msg-slot-value (achieve `(pose-estimated ,classified-object)) 'MPE_OBJECT))))))))))))))))
  resp))

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
  (let ((ids (get-yaml-object-nrs (roslisp:msg-slot-value environment:*yaml* 'objects) (roslisp:msg-slot-value (roslisp:msg-slot-value ?object 'object) 'id))))
    (perform (make-designator 'action `((to pose-estimate-object) (ids ,ids))))))

(defun get-yaml-object-nrs(yaml-objects  object-id)
  (let ((nrs (list))
        (i 0))
    (loop for object across yaml-objects do
          (if (string= (roslisp:msg-slot-value object 'name) object-id)
              (setq nrs (append nrs (list i))) )
          (incf i))
    nrs))

(def-goal (achieve (objects-in-place ?objects))
  (let ((target-zones (get-target-zones)))
    (mapcar (lambda (euroc-object)
              (ros-info (objects-in-place) "Processing object ~a" euroc-object)
              (let ((object-desig (make-designator 'object `((collision-object ,euroc-object)
                                                             (type ,(msg-slot-value euroc-object 'id))))))
                (ros-info (objects-in-place) "Created Designator ~a" object-desig)
                (let ((matching-target-zone (find-matching-target-zone object-desig target-zones)))
                  (ros-info (objects-in-place) "Matching target zone: ~a" matching-target-zone)
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
                        (ros-info (objects-in-place) "Achieving object-in-hand")
                        (achieve `(object-in-hand ,object-desig))
                        (ros-info (objects-in-place) "Done object-in-hand")
                        (equate (current-desig object-desig)
                                (copy-designator (current-desig object-desig)
                                                 :new-description `((at ,(make-designator 'location '((gripper gripper))))))))
                      (with-failure-handling
                        (((or manipulation-failure
                              manipulation-pose-unreachable) (e)
                          (declare (ignore e))
                          (ros-warn (objects-in-place) "Failed to put down object.")
                          (do-retry put-down-retry-count
                            (ros-warn (objects-in-place) "Retrying.")
                            (retry))))
                        (achieve `(object-placed-at ,(current-desig object-desig) ,matching-target-zone))
                        (equate (current-desig object-desig)
                                (copy-designator (current-desig object-desig)
                                                 :new-description `((at ,matching-target-zone))))))))))
            ?objects)))
