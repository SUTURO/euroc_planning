(in-package :planlib)

(defun classify()
  (setf-msg (value *taskdata*) (sec_try) nil)
  (let ((found-objects-gripper (perception:get-gripper-perception)))
    ))

   
(def-cram-function state-classify-objects ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-scan-obstacles) (eql *current-transition* :transition-new-image)))
      (print "Executing classify objects")
      (setf (value *current-state*) :state-classify-objects)
      (classify)
      (setf (value *current-transition*) :transition-todo)))
