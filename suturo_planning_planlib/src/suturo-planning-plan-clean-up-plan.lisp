(in-package :planlib)

(defparameter *target-zones* (make-array 0 :fill-pointer 0 :adjustable t))
(defparameter *target-zone-for-object* (make-hash-table :test 'equal))
(defparameter *clean-up-plan* (make-array 0 :fill-pointer 0 :adjustable t))
(defparameter *objects-in-target-zones* (make-array 0 :fill-pointer 0 :adjustable t))
(defparameter *taskdata-backup* NIL)
(defparameter *debug-continue* NIL)

(defun clean-up-plan()
  "Determine the order in which the found objects will be tidied up."
  (get-target-zones-from-userdata)
  (map-target-zones-to-object)
  (get-objects-located-in-target-zones)
  (create-plan)
  ;; here the objects should be sorted
)

(defun debug-stop()
  (wait-for *debug-continue*)
)

(defun save-taskdata()
  (setf *taskdata-backup* *taskdata*)
)

(defun restore-clean-up-plan()
  (init-clean-up-plan)
  (setf *taskdata* *taskdata-backup*)
)

(defun get-target-zones-from-userdata()
  (setf *target-zones* (cl-utilities:copy-array(roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'yaml) 'target_zones))))

(defun get-found-objects-from-userdata()
  (setf *found-objects* (cl-utilities:copy-array(roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'yaml) 'target-zones))))

(defun map-target-zones-to-object()
  (loop for target-zone across *target-zones* do
    (let (expected-object)
      (setf expected-object (roslisp:msg-slot-value (value target-zone) 'expected_object))
      (setf (gethash expected-object *target-zone-for-object*) target-zone))))

(defun get-objects-located-in-target-zones()
  (let ((target-zone))
  (loop for euroc-obj across *found-objects* do
    (print euroc-obj)
    (setf target-zone (in-target-zone euroc-obj))
    (if target-zone
        (if (not (is-object-expected euroc-obj target-zone))
            (vector-push-extend *objects-in-target-zones* target-zone))))))

(defun is-object-expected(euroc-obj target-zone)
  (let ((expected-object (roslisp:msg-slot-value (value target-zone) 'expected-object))
        (obj_id (roslisp:msg-slot-value (roslisp:msg-slot-value (value euroc-obj) 'mpe_object) 'id)))
    (if (string-equal obj_id expected-object)
        t nil)))

(defun in-target-zone(euroc-obj)
  (let ((obj-centroid (roslisp:msg-slot-value (value euroc-obj) 'c_centroid))
        (target-zone-centroid)
        (dist 0)
        (max-distance 0))
    (setf-msg obj-centroid (z) 0)
    (loop named distance-loop for target-zone across *target-zones* do
      (setf target-zone-centroid (roslisp:msg-slot-value (value target-zone) 'target_position))
      (setf dist (get-euclidean-distance obj-centroid target-zone-centroid))
      (setf max-distance (roslisp:msg-slot-value (value target-zone) 'max_distance))
        (if (< dist max-distance)
            (return-from distance-loop target-zone)
            NIL))))

(defun get-euclidean-distance(centroid1 centroid2)
  (let ((dist 0))
    (setf dist (sqrt (+ (expt (- (roslisp:msg-slot-value centroid1 'x) (roslisp:msg-slot-value centroid2 'x)) 2) 
                        (expt (- (roslisp:msg-slot-value centroid1 'y) (roslisp:msg-slot-value centroid2 'y)) 2)
                        (expt (- (roslisp:msg-slot-value centroid1 'z) (roslisp:msg-slot-value centroid2 'z)) 2))))))


(defun create-plan()
  (let ((plan-element (make-array 2 :fill-pointer 0)))
  (if (= (length *objects-in-target-zones*) 0)
      (loop for obj across *found-objects* do
        (vector-push obj plan-element)
        (vector-push (get-pose obj) plan-element)
        (vector-push-extend plan-element *clean-up-plan*)
        (print "Added to plan:")
        (print (get-id obj))))))


(defun get-id(obj)
  (roslisp:msg-slot-value (roslisp:msg-slot-value (value obj) 'mpe_object) 'id))

(defun get-pose(obj)
  (let ((obj_id (get-id obj))
        (target-position))
    (setf target-position (roslisp:msg-slot-value (value (gethash obj_id *target-zone-for-object*)) 'target_position))
    (make-msg "geometry_msgs/PointStamped" (header) (create-header) (point) target-position)    
))

(defun create-header()
  (make-msg "std_msgs/Header" (frame_id) "odom_combined")
)

(def-cram-function state-clean-up-plan ()
    (loop while T do
      (cpl-impl:wait-for  (fl-or
                          (fl-and (eql *current-state* :state-scan-obstacles) (eql *current-transition* :transition-no-region-left)) 
                          (fl-and (eql *current-state* :state-search-objects) (eql *current-transition* :transition-no-objects-left))
                          (fl-and (eql *current-state* :state-choose-object) (eql *current-transition* :transition-retry))
                          ))
      (print "Executing clean-up-plan")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-clean-up-plan)
      (clean-up-plan)
      (if (> (length *target-zones*) 0)
          (setf (value *current-transition*) :transition-success )
          (setf (value *current-transition*) :transition-fail))))
