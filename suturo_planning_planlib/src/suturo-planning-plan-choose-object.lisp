(in-package :planlib)

(defparameter *object-to-move* NIL)
(defparameter *objects-handled* 0)

(defun handle-choose-object()
  (choose-object)
  (increment-objects-handled)
  )

(defun choose-object()
  (setf *object-to-move* (aref *clean-up-plan* *objects-handled*))
  )

(defun increment-objects-handled()
  (incf *objects-handled*)
  )

(def-cram-function state-choose-object ()
  (loop while T do
      (cpl-impl:wait-for  (fl-or
                           (fl-and (eql *current-state* :state-clean-up-plan) (eql *current-transition* :transition-success)) 
                           (eql *current-state* :state-check-placement)
                           ))
      (print "Executing choose-object")
      (setf (value *current-transition*) :transition-nil)
      (setf (value *current-state*) :state-choose-object)
      (handle-choose-object)
      (if (= (length *clean-up-plan*) *objects-handled*)
          (setf (value *current-transition*) :transition-success )
          (setf (value *current-transition*) :transition-object-chosen))))
