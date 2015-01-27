(in-package :planlib)

(defparameter *target-zones* nil "")
(defparameter *target-zone-for-object* (make-hash-table :test 'equal))
(defparameter *found-objects* (make-array 0 :fill-pointer 0 :adjustable t) "A vector that contains every found object")

(defun handle-choose-object()
  "Determine the order in which the found objects will be tidied up."
  (print "handle choose object")
)

(defun get-target-zones-from-userdata()
  (*target-zones* (cl-utilities:copy-array(roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'yaml) 'target_zones))))
)

(defun map-target-zones-to-object()
  (loop for target-zone across *target-zones* do
    (let expected_object 
      (setf (expected_object (roslisp:msg-slot-value (value target-zone) 'expected_object)))
      (setf (gethash expected_object *target-zones-for-object*) target-zone))))