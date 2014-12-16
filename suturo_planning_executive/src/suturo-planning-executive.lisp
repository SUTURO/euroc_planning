(in-package :exec)

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
  (with-designators
    ((obj-loc (location `((pose ,(cl-tf:pose->pose-stamped "/map"
                                                           (ros-time)
                                                           (cl-transforms:make-identity-pose))))))
     (obj (object `((at ,obj-loc)
                    (type cube))))
     (obj-in-hand (object `((at ,(make-designator 'location '((gripper gripper)))))))
     (put-down-location (location `((pose ,(cl-tf:pose->pose-stamped "/map"
                                                                     (ros-time)
                                                                     (cl-transforms:make-identity-pose))))))
     (grasp-position (location '((to grasp)))))
    (with-process-modules
      (at-location (grasp-position)
        (achieve `(object-in-hand ,obj))
        (equate obj obj-in-hand) ; object is now in gripper
        (achieve `(object-put ,obj ,put-down-location))))))
