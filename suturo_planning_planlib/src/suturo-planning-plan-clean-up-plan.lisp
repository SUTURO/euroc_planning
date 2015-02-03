(in-package :planlib)

(defparameter *target-zones* NIL)
(defparameter *target-zone-for-object* NIL)
(defparameter *found-objects* NIL "A vector that contains every found object")
(defparameter *clean-up-plan* NIL)
(defparameter *objects-in-target-zones* NIL)
(defparameter *taskdata-backup* NIL)
(defparameter *debug-continue* NIL)

(defun clean-up-plan()
  "Determine the order in which the found objects will be tidied up."
  (print "handle clean up plan")
  (save-taskdata)
  (init-clean-up-plan)
  (debug-stop)
  (get-target-zones-from-userdata)
  (debug-stop)
  (map-target-zones-to-object)
  (debug-stop)
  (get-objects-located-in-target-zones)
  (debug-stop)
)

(defun debug-stop()
  (wait-for *debug-continue*)
  (setf *debug-continue* NIL)
)

(defun init-clean-up-plan()
  (setf *target-zones* (make-array 0 :fill-pointer 0 :adjustable t))
  (setf *target-zone-for-object* (make-hash-table :test 'equal))
  (setf *found-objects* (make-array 0 :fill-pointer 0 :adjustable t))
  (setf *clean-up-plan* (make-array 0 :fill-pointer 0 :adjustable t))
  (setf *objects-in-target-zones* (make-array 0 :fill-pointer 0 :adjustable t))
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
      (setf expected-object (roslisp:msg-slot-value (value target-zone) 'expected-object))
      (setf (gethash expected-object *target-zone-for-object*) target-zone))))

(defun get-objects-located-in-target-zones()
  (let ((target-zone))
  (loop for euroc-obj across *found-objects* do
    (setf target-zone (in-target-zone euroc-obj))
    (if target-zone
        (if (not (is-object-expected euroc-obj target-zone))
            (append *target-zones* target-zone))))))

(defun is-object-expected(euroc-obj target-zone)
  (let ((expected-object (roslisp:msg-slot-value (value target-zone) 'expected-object))
        (obj_id (roslisp:msg-slot-value (roslisp:msg-slot-value (value euroc-obj) 'mpe_object) 'id)))
    (if (string-equal obj_id expected-object)
        t nil)))

(defun in-target-zone(euroc-obj)
  (let ((obj-centroid (roslisp:msg-slot-value (value euroc-obj) 'c-centroid))
        (target-zone-centroid)
        (dist 0)
        (max-distance 0))
    (setf-msg (value obj-centroid) (z) 0)
    (loop named distance-loop for target-zone across *target-zones* do
      (setf target-zone-centroid (roslisp:msg-slot-value (value target-zone) 'target-position))
      (setf dist (get-euclidean-distance obj-centroid target-zone-centroid))
      (setf max-distance (roslisp:msg-slot-value (value target-zone) 'max-distance))
        (if (< dist max-distance)
            (return-from distance-loop target-zone)
            NIL))))

(defun get-euclidean-distance(centroid1 centroid2)
  (let ((dist 0))
    (setf dist (sqrt (+ (expt (- (roslisp:msg-slot-value centroid1 'x) (roslisp:msg-slot-value centroid2 'x)) 2) 
                        (expt (- (roslisp:msg-slot-value centroid1 'y) (roslisp:msg-slot-value centroid2 'y)) 2)
                        (expt (- (roslisp:msg-slot-value centroid1 'z) (roslisp:msg-slot-value centroid2 'z)) 2))))))


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
      (setf (value *current-transition*) (clean-up-plan))))
