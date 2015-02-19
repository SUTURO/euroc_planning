(in-package :exec)

(defun parse-yaml ()
  ; TODO: Publish the yaml description to the yaml pars0r input
  (roslisp:subscribe constants:+topic-name-get-yaml+ 'suturo_msgs-msg:Task #'yaml-cb))

(roslisp-utilities:register-ros-init-function parse-yaml)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
     (suturo-planning-pm-manipulation
      suturo-planning-pm-perception)
     ,@body))

(defmacro with-ros-node (&body body)
  "Utility macro to start and stop a ROS node around a block"
  `(prog2
     (roslisp-utilities:startup-ros)
     ,@body
     (roslisp-utilities:shutdown-ros)))

(defun task-selector (&optional (tsk "task1"))
  "Starts the plan for the task from the parameter server.

   If no task is set in the parameter server, the task given
   as argument will be started (default: task1)"
  (with-ros-node
    (let ((task (roslisp:get-param "/planning/task" tsk)))
      (funcall (symbol-function (read-from-string (format nil "exec:~a" task)))))))

(def-top-level-cram-function task1 ()
  "Top level plan for task 1 of the euroc challenge"
  (with-process-modules
    (with-retry-counters ((all-retry-count 2)
                          (scan-map-retry-count 2)
                          (inform-objects-retry-count 2)
                          (objects-in-place-retry-count 3))
      (with-failure-handling
        ((simple-plan-failure (e)
           (declare (ignore e))
           (ros-warn (toplevel task1) "Something failed.")
           (do-retry all-retry-count
             (ros-warn (toplevel task1) "Retrying all.")
             (reset-counter scan-map-retry-count)
             (reset-counter inform-objects-retry-count)
             (reset-counter objects-in-place-retry-count)
             (retry))))
        (with-failure-handling
          (((or map-scanning-failed moving-mast-cam-failed) (e)
             (declare (ignore e))
             (ros-warn (toplevel task1) "Failed to scan map.")
             (do-retry scan-map-retry-count
               (ros-warn (toplevel task1) "Retrying.")
               (retry))))
          (achieve '(map-scanned))
          (with-failure-handling
            ((objects-information-failed (e)
               (declare (ignore e))
               (ros-warn (toplevel task1) "Failed to inform objects.")
               (do-retry inform-objects-retry-count
                 (ros-warn (toplevel task1) "Retrying.")
                 (retry))))
            (let ((objects (achieve '(objects-informed))))
              (with-failure-handling
                (((or objects-in-place-failed
                      manipulation-failure) (e)
                   (declare (ignore e))
                   (ros-warn (toplevel task1) "Failed to put objects in place.")
                   (do-retry objects-in-place-retry-count
                     (ros-warn (toplevel task1) "Retrying.")
                     (retry))))
                (achieve `(objects-in-place ,objects))))))))))


(def-top-level-cram-function task1-tmp ()
  (roslisp:with-ros-node "testExecution"
  (with-process-modules
    (cpl-impl:par
      (planlib:do-planning "task1_v1")
      (progn
        (print "Waiting")
        (cpl-impl:wait-for (fl-and (eql *current-state* :state-init) (eql *current-transition* :transition-successful)))
        (roslisp:subscribe constants:+topic-name-get-yaml+ 'suturo_msgs-msg:Task #'yaml-cb)
        (task1)     
        ;(setf (value *current-state*) :state-scan-shadow)
        ;(setf (value *current-transition*) :transition-success)
        )))))

(defun yaml-cb (msg)
  (setf environment:*yaml* msg))
