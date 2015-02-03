(in-package :planlib)

(defun handle-choose-object()
  "Chooses an object from the clean up plan that should be tidied up next."
  (print "handle choose object")
  (save-taskdata)
  (init-choose-object)
  (get-clean-up-plan)
  (print (length *clean-up-plan*))
)

(defun init-choose-object()
  (defparameter *clean-up-plan* (make-array 0 :fill-pointer 0 :adjustable t))
)

(defun save-taskdata()
  (defparameter *taskdata-backup *taskdata*)
)

(defun restore-choose-object()
  (init-choose-object)
  (setf *taskdata* *taskdata-backup*)
)

(defun get-clean-up-plan()
  (setf *clean-up-plan* (cl-utilities:copy-array(roslisp:msg-slot-value (value *taskdata*) 'clean_up_plan))))

