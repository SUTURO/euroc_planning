(in-package :planlib)

;;Vll die Objekte aufgrund der id's vergleichen anstatt der kompletten objekte ?
;;TODO MISSING OBJECTS NOT DELETED !?!?!?!

(defparameter *missing-objects* nil "A vector that contains every object that hasnt been found yet")
(defparameter *found-objects* (make-array 0 :fill-pointer 0 :adjustable t) "A vector that contains every found object")

(defun add-yaml-target-objects()
  "Adds the object descriptions from the yaml files to the *missing-objects* vector" 
  (if (eql *missing-objects* nil)
      (let ((yaml-objects (cl-utilities:copy-array(roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'yaml) 'objects))))
        (setf *missing-objects* (make-array 0 :fill-pointer 0 :adjustable t))
        (loop for i from 0 upto (- (length yaml-objects) 1) do
          (vector-push-extend (elt yaml-objects i) *missing-objects*)))))
               
(defun handle-found-objects()
  "Checks if a new object was found. The vector fitted_objects from *taskdata* contains every found object. If the vector fitted-objects contains an object which isn't in *found-objects*,
   the object is added to *found-objects* and removed from *missing-objects*."
  (print "handle found objects")
  (print (length (roslisp:msg-slot-value (value *taskdata*) 'fitted_objects)))
  (sleep 30) ;; Warum muss ich hier warten ?? Wenn nicht gewartet wird, ist beim zweiten Aufruf von search object das "fitted_objects" noch mit keinem Objekt belegt. Die For-Schleife wird umgangen
  (print (length (roslisp:msg-slot-value (value *taskdata*) 'fitted_objects)))
 (let ((fitted-objects (roslisp:msg-slot-value (value *taskdata*) 'fitted_objects))) 
    ;;Check if the vector fitted-objects contains objects that aren't already in *found-objects*
    (loop for fitted-object across fitted-objects do
      (if (vector-contains-obj *found-objects* fitted-object)
          (print "object already found")
          (progn
            (print "New object found")
            (vector-push-extend fitted-object *found-objects*)
            (vector-remove-object *missing-objects* fitted-object)))))
  (setf-msg (value *taskdata*) (objects_found) *found-objects*)
  (print "end handle found objects"))

(defun vector-contains-obj(vec obj)
"Checks if the vector contains the object. Return T / nil"
  (print "vector contains")
  (loop for vec-obj across vec do
    (if (equal-message vec-obj obj)
        (return T))))

(defun vector-remove-object(vec obj)
  "Removes the object from a vector, if the vector contains the object"
  (print "vector removes")
  (let ((counter (vector-get-position vec obj)) (tmp (make-array 0 :fill-pointer 0 :adjustable t)))
    (if (not (eql counter nil))
        (progn
          (loop for i from 0 upto counter do
            (vector-push-extend (vector-pop vec)  tmp))
          (vector-pop vec)
          (loop for i from counter downto 0 do
            (vector-push (elt tmp i) vec))))))

(defun vector-get-position(vec obj)
 "Returns the position of an object in the vector or nil, if the vector doesn't contain the object. The counting start at 0."
  (print "vector get position")
  (let ((counter 0))
    (loop for vec-obj across vec do
      (if (equal-message vec-obj obj)
          (return counter)
          (incf counter)))))
          
(defun equal-message (msg1 msg2)
  "tests equalness of messages by comparing print output src: https://code.ros.org/trac/ros/browser/stacks/roslisp_support/trunk/test_roslisp/src/test-roslisp/topic-tests.lisp?rev=14957"
  (equal (format nil "~a" msg1) (format nil "~a" msg2)))

(def-cram-function state-search-objects ()
    (loop while T do
      (cpl-impl:wait-for (fl-or (fl-and (eql *current-state* :state-scan-shadow) (eql *current-transition* :transition-success))
                                (fl-and (eql *current-state* :state-classify-objects) (eql *current-transition* :transition-no-object)) 
                                (fl-and (eql *current-state* :state-focus-objects) (eql *current-transition* :transition-success) )
                                (fl-and (eql *current-state* :state-focus-objects) (eql *current-transition* :transition-fail))))
      (print "Executing state search objects")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-search-objects)
      (add-yaml-target-objects)
      (handle-found-objects)
      (if (= (length *missing-objects*) 0)
          (setf (value *current-transition*) :transition-no-objects-left)
          (setf (value *current-transition*) :transition-missing-objects))))
