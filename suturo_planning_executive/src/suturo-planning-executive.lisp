(in-package :exec)

(defvar *yaml-pub*)

(defun parse-yaml ()
  "Subscribes the yaml publisher and sets environment:*yaml* to stay informed about changes"
  ; TODO: Publish the yaml description to the yaml pars0r input
  (roslisp:subscribe constants:+topic-name-get-yaml+ 'suturo_msgs-msg:Task #'yaml-cb))

(defun yaml-publisher ()
  "Creates the publisher for the yaml-file"
  (setf *yaml-pub* (advertise "/suturo/yaml_pars0r_input" "std_msgs/String")))

(roslisp-utilities:register-ros-init-function parse-yaml)
(roslisp-utilities:register-ros-init-function yaml-publisher)

(defmacro with-process-modules (&body body)
  "Macro to define the used process modules."
  `(cpm:with-process-modules-running
       (suturo-planning-pm-manipulation
        suturo-planning-pm-perception)
     ,@body))

(defmacro with-ros-node (&body body)
  "Utility macro to start and stop a ROS node around a block"
  `(unwind-protect
    (progn
      (roslisp-utilities:startup-ros)
      ,@body)
    (roslisp-utilities:shutdown-ros)))

(defun task-selector ()
  "Starts the plan for the task from the parameter server.

   If no task is set in the parameter server, the task given
   as argument will be started (default: task1_v1)"
  (with-ros-node
    (let ((tsk (get-param "/task_variation" "task1_v1")))
      (unless (wait-for-service +service-name-start-simulator+ +timeout-service+)
        (ros-error (task-selector) "Service ~a timed out." +service-name-start-simulator+)
        (fail))
      (ros-info (task-selector) "Starting simulator...")
      (let* ((ret (call-service +service-name-start-simulator+
                                            'euroc_c2_msgs-srv:StartSimulator
                                            :user_id "suturo"
                                            :scene_name tsk))
             (_ (ros-info (task-selector) "Task starter return: ~a" ret))
             (task-description (slot-value ret 'euroc_c2_msgs-srv:description_yaml)))
        (publish-msg *yaml-pub* :data task-description)
        (let ((task (remove #\  (get-param "/task_description/public_description/task_name" tsk)))) ; whitespace sensitive!
          (ros-info (task-selector) "Starting plan ~a..." task)
          (unwind-protect
            (print "funcall")
            (defparameter my-task task)
            (funcall (symbol-function (read-from-string (format nil "exec:~a" task))))
            (print "funcall done")
            (when cram-beliefstate::*logging-enabled*
              (ros-info (task-selector) "Saving log files...")
              (cram-beliefstate:extract-files))))))))

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
                        cram-plan-failures:manipulation-failure) (e)
                     (declare (ignore e))
                     (ros-warn (toplevel task1) "Failed to put objects in place.")
                     (do-retry objects-in-place-retry-count
                       (ros-warn (toplevel task1) "Retrying.")
                       (retry))))
                (achieve `(objects-in-place ,objects))))))))))

(defun task1-tmp (&optional start_sim)
  "
Temporary top-level plan to start the task 1. The argument *start\_sim* should be T if the function is called the first time. Set the argument to nil
if the plan should try to continue from the last state.
* Arguments
- start\_sim :: T if the simulation should be started
"
  (roslisp:with-ros-node "testExecution"
    (if start_sim
        (cram-task1-tmp t)
        (cram-task1-tmp))))

(def-top-level-cram-function cram-task1-tmp (&optional start_sim)
  "
Temporary top-level plan to start the task 1. The argument *start\_sim* should be T if the function is called the first time. Set the argument to nil
if the plan should try to continue from the last state.
* Arguments
- start\_sim :: T if the simulation should be started
"
  (with-process-modules
    (if start_sim
        (init-simulation "task1_v1"))
    (roslisp:subscribe constants:+topic-name-get-yaml+ 'suturo_msgs-msg:Task #'yaml-cb)
    (achieve `(suturo-planning-planlib::map-scanned))
    (let ((objects (achieve `(objects-informed))))
      (achieve `(objects-in-place ,objects)))))

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
    (sleep 20)
    (call-service-state "init" taskdata)) 
  (manipulation:init))
