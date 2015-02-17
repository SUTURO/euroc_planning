(in-package :planlib)

(defun pose-estimate ()
  (setf-msg (value *taskdata*) (focused_object) (roslisp:msg-slot-value (value *taskdata*) 'object_to_focus))
  (setf-msg (value *taskdata*) (sec_try) nil)
  (setf-msg (value *taskdata*) (fitted_object) nil) ;;Verursacht vll einen Fehler. Leeres Objekt ist besser ? Wie sieht die Startbesetzung aus ?
  (let ((ids (get-yaml-object-nrs (roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'yaml) 'objects) (roslisp:msg-slot-value (roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'focused_object) 'object) 'id)))
        (transition nil))
    (let ((pose-estimated-objects (roslisp:msg-slot-value (perception:get-gripper-perception nil T ids) 'objects)))
      (if (= (length pose-estimated-objects) 0)
          (progn
            (if (not (roslisp:msg-slot-value (value *taskdata*) 'sec_try_done))
                (setf-msg (value *taskdata*) (sec_try) T))
            (setf transition :transition-fail))
          (progn
            (setf pose-estimated-objects (convert-euroc-objects-to-odom-combined pose-estimated-objects))
            (let ((corresponding-object-idx (get-nearest-object-idx (roslisp:msg-slot-value (value *taskdata*) 'focused_object) pose-estimated-objects 0.1) )
                  (pose-estimated nil))
              (if (not corresponding-object-idx)
                  (progn
                    (if (not (roslisp:msg-slot-value (value *taskdata*) 'sec_try_done))
                        (setf-msg (value *taskdata*) (sec_try) T))
                    (print "HERE")
                    (setf transition :transition-fail))
                  (progn
                    (setf pose-estimated (elt pose-estimated-objects corresponding-object-idx))
                    (format t "~a" pose-estimated)
                    (if (or (not pose-estimated) (not (roslisp:msg-slot-value pose-estimated 'mpe_success)))
                        (progn
                          (if (not (roslisp:msg-slot-value (value *taskdata*) 'sec_try_done))
                              (setf-msg (value *taskdata*) (sec_try) T))
                          (print "THERE")
                          (setf transition :transition-fail))
                        (progn
                          (print "Im already right here")
                          (setf-msg (value *taskdata*) (fitted_object) pose-estimated)
                          (call-add-collision-objects (vector (roslisp:msg-slot-value pose-estimated 'mpe_object)))
                          (setf transition :transition-success)))))))))
    transition))



(defun get-yaml-object-nrs(yaml-objects  object-id)
  (let ((nrs (list))
        (i 0))
    (loop for object across yaml-objects do
          (if (string= (roslisp:msg-slot-value object 'name) object-id)
              (setq nrs (append nrs (list i))) )
          (incf i))
    nrs))


(defun get-nearest-object-idx(obj objects treshold)
  (let ((min-distance 999999999999999) ;;infinity
        (min-distance-idx 0)
        (i 0)
        (dist 0)
        (obj-centroid (roslisp:msg-slot-value obj 'c_centroid)))
    (loop for object across objects do
      (let ((object-centroid (roslisp:msg-slot-value object 'c_centroid)))
        (setf dist (sqrt (+ (expt (- (roslisp:msg-slot-value obj-centroid 'x) (roslisp:msg-slot-value object-centroid 'x)) 2) 
                            (expt (- (roslisp:msg-slot-value obj-centroid 'y) (roslisp:msg-slot-value object-centroid 'y)) 2)
                            (expt (- (roslisp:msg-slot-value obj-centroid 'z) (roslisp:msg-slot-value object-centroid 'z)) 2)))) 
        (if (< dist min-distance)
            (progn
              (setf min-distance dist)
              (setf min-distance-idx i))))
      (incf i))
    (if (< min-distance treshold)
        min-distance-idx
        nil)))
      
    

(def-cram-function state-pose-estimate-object ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-focus-objects) (fl-or (eql *current-transition* :transition-focus-handle) (eql *current-transition* :transition-focus-object))))
      (print "Executing state pose estimate object")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-pose-estimate-object)
      (setf (value *current-transition*) (pose-estimate))))
