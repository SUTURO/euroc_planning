(in-package :planlib)

(def-goal (achieve (map-scanned))
    "Tries to scan the whole map."
  (scan-map-mast-cam)
  (scan-shadow))

(def-goal (achieve (objects-informed))
    "Document me"
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
    "* Arguments
- ?objects :: An array of suturo_perception_msgs-msg:EurocObjects which should be found in the map 
* Return Value 
Returns every found region as in the map where the objects are suspected. The return type is an array of suturo_environment_msgs-msg:Region. 
* Description 
Tries to find the given objects in the current map"
  (perform (make-designator 'action `((to find-objects-in-map) 
                                      (objects ,?objects)))))

(def-goal (achieve (my-test-goal '(?param1)))
    "* Arguments
- ?objects :: An array of suturo_perception_msgs-msg:EurocObjects which should be found in the map 
* Return Value 
Returns every found region as in the map where the objects are suspected. The return type is an array of suturo_environment_msgs-msg:Region. 
* Description 
Tries to find the given objects in the current map"
                                     nil )

(def-goal (achieve (unknown-scanned ?region))
    "* Arguments
- ?region :: A suturo_environment_msgs-msg:Region which should be scanned
*Return Value
Returns the suturo_perception_msgs-msg:EurocObject recognized by the scene cam
* Description
Tries to scan and recognize an object in the region. Moves the arm-cam in the proper position to scan the region"
  (look-at-obstacle ?region)
  (roslisp:msg-slot-value (perform (make-designator 'action `((to get-gripper-perception)))) 'objects))

(def-goal (achieve (object-classified ?object))
"* Arguments
- ?object :: A suturo_perception_msgs-msg:EurocObject which should be classified
*Return Value
TODO
* Description
Tries to classify the given object"
  (let ((obj (perform (make-designator 'action `((to classify-object)
                                                 (obj ,?object))))))
    (if obj
        obj
        (progn
          (ros-warn (achieve object-classified) "Failed to classify object ~a" ?object)
          (fail)))))

(def-goal (achieve (pose-estimated ?object))
"* Arguments
- ?object :: A suturo_perception_msgs-msg:EurocObject which pose should be estimated
*Return Value
TODO
* Description
Tries to estimate the pose of the given object"
  (let ((ids (get-yaml-object-nrs (roslisp:msg-slot-value environment:*yaml* 'objects) (roslisp:msg-slot-value (roslisp:msg-slot-value ?object 'object) 'id))))
    (perform (make-designator 'action `((to pose-estimate-object) (ids ,ids))))))

(defun get-yaml-object-nrs(yaml-objects  object-id)
"* Arguments
- yaml-objects :: The array of suturo_perception_msgs-msg:EurocObjects to search through
- object-id :: The object-id / name of an object as string
*Return Value
The positions of the suturo_perception_msgs-msg:EurocObjects in the array /yaml-objects/ with the id /object-id/
* Description
Searches through the /yaml-objects/ and returns a list of positions with the id /object-id/"
  (let ((nrs (list))
        (i 0))
    (loop for object across yaml-objects do
          (if (string= (roslisp:msg-slot-value object 'name) object-id)
              (setq nrs (append nrs (list i))) )
          (incf i))
    nrs))

(def-goal (achieve (objects-in-place ?objects))
    "DOCUMENT ME"
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
