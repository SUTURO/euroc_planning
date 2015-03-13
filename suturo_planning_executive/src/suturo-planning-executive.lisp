(in-package :exec)

(defun parse-yaml ()
  "Subscribes the yaml publisher and sets environment:*yaml* to stay informed about changes"
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

(defun task-selector (&optional (tsk "task1_v1"))
  "Starts the plan for the task from the parameter server.

   If no task is set in the parameter server, the task given
   as argument will be started (default: task1)"
  (with-ros-node
    (let ((task (remove #\  (roslisp:get-param "/task_variation/task_name" tsk)))) ; whitespace sensitive!
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


(def-top-level-cram-function task1-tmp (&optional start_sim)
  "
Temporary top-level plan to start the task 1. The argument *start_sim* should be T if the function is called the first time. Set the argument to nil
if the plan should try to continue from the last state.
* Arguments
- start_sim :: T if the simulation should be started
"
  (roslisp:with-ros-node "testExecution"
  (with-process-modules
    (if start_sim
        (init-simulation "task1_v1"))
    (roslisp:subscribe constants:+topic-name-get-yaml+ 'suturo_msgs-msg:Task #'yaml-cb)
    (achieve `(suturo-planning-planlib::map-scanned))
    (let ((objects (achieve `(objects-informed))))
      (achieve `(objects-in-place ,objects))))))

(defun yaml-cb (msg)
  "
Callback for the function [[parse-yaml]]. Sets the variable environment:*yaml*.
"
  (setf environment:*yaml* msg))

(defun call-service-state (service-name taskdata)
  "
Calls the service of the given service-name. Every state service has to accept an object of suturo_interface_msgs-srv:TaskDataService.
* Arguments
- service-name :: The name of a state service has to start with suturo/state/. This argument needs the last part of the service name e.g: suturo/state/myAwesomeService -> myAwesomeService.
- taskdata :: The suturo_interface_msgs-msgs:Taskdata object that should be send to the service
"
  (let
      ((full-service-name (concatenate 'string "suturo/state/" service-name)))
    (print (concatenate 'string "calling service: " service-name))
    (if (not (roslisp:wait-for-service full-service-name +timeout-service+))
        (progn
          (let 
              ((timed-out-text (concatenate 'string "Timed out waiting for service " service-name)))
            (roslisp:ros-warn nil t timed-out-text))
          nil)
        (let ((value (roslisp:call-service full-service-name 'suturo_interface_msgs-srv:TaskDataService :taskdata taskdata)))
          (roslisp:msg-slot-value value 'result)))))

(def-cram-function init-simulation (task_name)
  "
Initialize the simulation:
- Start simulation
- Start manipulation
- Start perception
- Start classifier
"
  (let ((taskdata (roslisp:make-msg "suturo_interface_msgs/TaskData" (name) task_name)))
    (call-service-state "start_simulation" taskdata)
    (call-service-state "start_manipulation" taskdata)
    (call-service-state "start_perception" taskdata)
    (call-service-state "start_classifier" taskdata)
    (call-service-state "init" taskdata)) 
  (manipulation:init))
