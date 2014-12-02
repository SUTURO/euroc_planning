(in-package :exec)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
     (suturo-planning-pm-manipulation
      suturo-planning-pm-perception)
     ,@body))

(defun main ()
  "Main function that executes when the executable is run"
  (task1))

(def-top-level-cram-function task1 ()
  "Top level plan for task 1 of the euroc challenge"
  (with-process-modules
    nil))
