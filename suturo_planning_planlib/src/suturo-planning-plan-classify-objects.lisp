(in-package :planlib)

(defconstant +object-type-table+ 0 "")
(defconstant +object-type-object+ 1 "")
(defconstant +object-type-obstacle+ 2 "")
(defconstant +object-type-unknown+ 255 "")

(defun classify()
  (let ((perceived-objects (roslisp:msg-slot-value (perception:get-gripper-perception) 'objects))
        (found-object-names (get-found-object-names) )
        (collision-objects (make-array 0 :fill-pointer 0 :adjustable t))
        (matched-objects (make-array 0 :fill-pointer 0 :adjustable t))
        (transition nil))
    (setf-msg (value *taskdata*) (sec_try) nil)
    (if (check-objects-found perceived-objects) ;;TODO Change name
        (progn
          (analyze-perceived-objects perceived-objects found-object-names collision-objects matched-objects)
          (setf-msg (value *taskdata*) (classified_objects) matched-objects)
          (if (> (length matched-objects) 0)
             (setf transition :transition-objects-classified)
             (progn
               (print "no matched objects found")
               (setf transition :transition-no-object)
               (if (not (roslisp:msg-slot-value (value *taskdata*) 'sec_try_done))
                   (setf-msg (value *taskdata*) (sec_try) T)))))
        (setf transition :transition-no-object))
    transition))


(defun check-objects-found(perceived-objects)
  (if (= (length perceived-objects) 0)
      (progn
        (if (not (roslisp:msg-slot-value (value *taskdata*) 'sec_try_done)) 
            (setf-msg (value *taskdata*) (sec_try) T))
        nil)
      T))


(defun get-found-object-names ()
  (let ((found-object-names (make-array 0 :fill-pointer 0 :adjustable t)))
    (loop for found-object across (roslisp:msg-slot-value (value *taskdata*) 'objects_found) do
      (vector-push-extend (roslisp:msg-slot-value (roslisp:msg-slot-value found-object 'mpe_object) 'id) found-object-names))
    found-object-names))


(defun analyze-perceived-objects(perceived-objects found-object-names collision-objects matched-objects)
  (loop for object across perceived-objects do
    (let ((matched-object (roslisp:msg-slot-value (call-classify-object object) 'classifiedObject)))
      (cond 
        ((= (roslisp:msg-slot-value matched-object 'c_type) +object-type-obstacle+) (handle-object-obstacle matched-object collision-objects))
        ((or (= (roslisp:msg-slot-value matched-object 'c_type) +object-type-unknown+) (= (roslisp:msg-slot-value matched-object 'c_type) +object-type-table+)) 
         (handle-object-unknown-or-table matched-object collision-objects))
        ((= (roslisp:msg-slot-value matched-object 'c_type) +object-type-object+) (handle-object matched-object found-object-names matched-objects)) ))))

    
(defun handle-object-obstacle (matched-object collision-objects)
  (if (roslisp:msg-slot-value matched-object 'c_cuboid_success)
      (progn
        (print "Found obstacle")
        (vector-push-extend matched-object collision-objects))))


(defun handle-object-unknown-or-table (matched-object collision-objects) 
  (if (roslisp:msg-slot-value matched-object 'c_cuboid_success)
      (progn
        (vector-push-extend matched-object collision-objects)
        (print "Unknown object or table perceived. Ignoring it for now."))))


(defun handle-object (matched-object found-object-names matched-objects)
  (print "Found object")
  (let ((id-already-found nil))
    (loop for name across found-object-names do
      (if (string= (roslisp:msg-slot-value (roslisp:msg-slot-value matched-object 'object) 'id) name )
          (progn
            (print "id already found")
            (setf id-already-found T))))
    (if (not id-already-found)
        (progn
          ;;Todo euroc-object-to-odom-combined
          (print "new object found")
          (vector-push-extend (roslisp:msg-slot-value (call-euroc-object-to-odom-combined matched-object)'converted) matched-objects)))))


(defun call-classify-object (object)
  (print "Calling classify object ")
  (if (not (roslisp:wait-for-service +service-name-classify-objects+ *timeout-service*))
      (progn 
        (print "Timed out")
        (setf (value *current-transition*) :transition-timed-out)) 
      (roslisp:call-service +service-name-classify-objects+ 'suturo_perception_msgs-srv:Classifier :unclassifiedObject object)))

;;Todo verschieben
(defun call-euroc-object-to-odom-combined(object)
  (print "Calling euroc object to odom combined ")
  (if (not (roslisp:wait-for-service +service-name-euroc-object-to-odom-combined+ *timeout-service*))
      (progn 
        (print "Timed out")
        (setf (value *current-transition*) :transition-timed-out)) 
      (roslisp:call-service +service-name-euroc-object-to-odom-combined+ 'suturo_interface_msgs-srv:EurocObjectToOdomCombined :toConvert object)))


(def-cram-function state-classify-objects ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-scan-obstacles) (eql *current-transition* :transition-new-image) ))
      (print "Executing state classify objects")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-classify-objects)
      (setf (value *current-transition*) (classify))))
