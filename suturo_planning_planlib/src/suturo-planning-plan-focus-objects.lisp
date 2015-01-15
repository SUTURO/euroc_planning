(in-package :planlib)

(defvar *objects-to-focus* nil)
(defvar *fitted-objects* (make-array 0 :fill-pointer 0 :adjustable t))


(defun focus-objects()
  (let ((transition nil))
    (setf-msg (value *taskdata*) (objects_to_focus) (roslisp:msg-slot-value (value *taskdata*) 'classified_objects))
    (if (not *objects-to-focus*)
        (progn
          (print "A")
          (setf *objects-to-focus* (roslisp:msg-slot-value (value *taskdata*) 'objects_to_focus))
          (print "B")
          (handle-change-object-to-focus))
        (progn
          (print "ASD")
          (remember-fitted-object)
          (print "ASDF")
          (handle-map-changes)))
    (print "C")
    (setf transition (handle-no-more-objects-to-focus))
    (print "D")
    (format t "~a" transition)
    (if (not transition) ;;still object to focus (handle-no-mor-objects-to-focus returned nil)
        (progn
          (set-next-object-to-focus)
          (print "E")
          (if (is-handle (roslisp:msg-slot-value (roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'object_to_focus) 'object) 'id) (roslisp:msg-slot-value (value *taskdata*) 'yaml))
              (setf transition :transition-focus-handle)
              (setf transition :transition-focus-object))))
    transition))


(defun set-next-object-to-focus ()
  (let ((next-object (vector-pop *objects-to-focus*)))
    (setf-msg (value *taskdata*) (object_to_focus) next-object)))


(defun handle-no-more-objects-to-focus()
  (if (= (length *objects-to-focus*) 0)
      (progn
        (setf *objects-to-focus* nil)
        (setf-msg (value *taskdata*) (fitted_objects) *fitted-objects*)
        (setf *fitted-objects* (make-array 0 :fill-pointer 0 :adjustable t))
        :transition-success)
      nil))


(defun handle-change-object-to-focus()
  (if (roslisp:msg-slot-value (value *taskdata*) 'focused_point)
      (let ((object-to-focus nil)
            (min-dist 100)
            (dist 0)
            (tmp-vector (make-array 1 :fill-pointer 0)))
        (loop for object across *objects-to-focus* do
          (setf dist (euclidean-distance (roslisp:msg-slot-value object 'c_centroid) (roslisp:msg-slot-value (value *taskdata*) 'focused_point))) 
              (if (< dist min-dist)
                  (progn
                    (setf min-dist dist)
                    (setf object-to-focus object))))
        (vector-push object-to-focus tmp-vector)
        (setf *objects-to-focus* tmp-vector))))


(defun euclidean-distance (p1 p2)
  (let ((dist 0))
    (setf dist (sqrt (+ (expt (- (roslisp:msg-slot-value p1 'x) (roslisp:msg-slot-value p2 'x)) 2) 
                        (expt (- (roslisp:msg-slot-value p1 'y) (roslisp:msg-slot-value p2 'y)) 2)
                        (expt (- (roslisp:msg-slot-value p1 'z) (roslisp:msg-slot-value p2 'z)) 2))))
    dist))


(defun remember-fitted-object ()
  (if (roslisp:msg-slot-value (value *taskdata*) 'fitted_object)
      (progn
        (vector-push-extend (roslisp:msg-slot-value (value *taskdata*) 'fitted_object) *fitted-objects*))))


(defun handle-map-changes ()
  (if (roslisp:msg-slot-value (value *taskdata*) 'fitted_object)
      (let ((position (roslisp:msg-slot-value (elt (roslisp:msg-slot-value (roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'fitted_object) 'mpe_object) 'primitive_poses) 0) 'position)))
        (call-mark-region-as-object-under-point (roslisp:msg-slot-value position 'x) (roslisp:msg-slot-value position 'y))
        (call-add-collision-objects (vector (call-current-map-to-collision-object))))))


(defun is-handle (name yaml)
  (let ((filter-handle (make-array 0 :fill-pointer 0 :adjustable t))
        (name-in-handle nil))
    (loop for object across (roslisp:msg-slot-value yaml 'objects) do
      (if (> (length (roslisp:msg-slot-value object 'primitives)) 1)
          (vector-push-extend object filter-handle)))
    (loop for handle across filter-handle do
      (if (string= (roslisp:msg-slot-value handle 'name) name)
          (setf name-in-handle T)))
    name-in-handle))


(defun call-current-map-to-collision-object()
  (print "Calling current map to collision object")
  (if (not (roslisp:wait-for-service +service-name-current-map-to-collision-object+ *timeout-service*))
      (progn 
        (print "Timed out")
        (setf (value *current-transition*) :transition-timed-out)) 
      (roslisp:msg-slot-value (roslisp:call-service +service-name-current-map-to-collision-object+ 'suturo_interface_msgs-srv:CurrentMapToCollisionObject) 'object)))


(defun call-mark-region-as-object-under-point(x y)
  (print "Calling mark region as object under point")
  (if (not (roslisp:wait-for-service +service-name-mark-region-as-object-under-point+ *timeout-service*))
      (progn 
        (print "Timed out")
        (setf (value *current-transition*) :transition-timed-out)) 
      (roslisp:msg-slot-value (roslisp:call-service +service-name-mark-region-as-object-under-point+ 'suturo_interface_msgs-srv:MarkRegionAsObjectUnderPoint :x x :y y) 'result)))


(def-cram-function state-focus-objects ()
    (loop while T do
      (cpl-impl:wait-for  (fl-or
                          (fl-and (eql *current-state* :state-classify-objects) (eql *current-transition* :transition-objects-classified)) 
                          (fl-and (eql *current-state* :state-pose-estimate-object) (fl-or (eql *current-transition* :transition-fail) (eql *current-transition* :transition-success)))))
      (print "Executing state focus objects")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-focus-objects)
      (setf (value *current-transition*) (focus-objects))))
