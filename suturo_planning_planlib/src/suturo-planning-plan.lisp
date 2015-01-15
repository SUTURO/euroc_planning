(in-package :planlib)

;;Keywords for the different states
:state-init
:state-scan-map
:state-scan-shadow
:state-search-objects
:state-scan-obstacles
:state-classify-objects
:state-focus-objects
:state-pose-estimate-object

;;Keywords for the different transitions
:transition-start
:transition-successful
:transition-map-scanned
:transition-success
:transition-missing-objects
:transition-no-objects-left
:transition-new-image
:transition-no-region-left
:transition-objects-classified
:transition-no-object
:transition-fail
:transition-focus-handle
:transition-focus-object
:transition-timed-out

(defstruct (state-data :conc-name)
  "Represents the position of the mast cam"
  (current-state :state-init)
  (current-transition :transition-start)
  (taskdata nil))

(defvar *current-state* (make-fluent :name :current-state :value :state-init)"The current state of the state machine")
(defvar *current-transition* (make-fluent :name :current-transition :value :transition-start) "The transition returned from a state")
(defvar *taskdata* (make-fluent :name :taskdata :value nil) "The needed data for the state machine")
(defvar *name-node* "statemachine" "The name of the ros-node")
(defvar *name-service-init* "suturo/toplevel/init" "The name of the init service")
(defvar *name-service-create-taskdata* "suturo/toplevel/create_task_data" "Name of the service to get an object of taskdata") 
(defvar *timeout-service* 10 "The time to wait for a service")
(defvar *state* (make-fluent :name :state) "The current state")

(defconstant +service-name-move-mastcam+ "/suturo/manipulation/move_mastcam" "The name of the service to move the mastcam")
(defconstant +service-name-add-point-cloud+ "/suturo/add_point_cloud" "The name of the service to add a point cloud")
(defconstant +service-name-move-robot+ "/suturo/manipulation/move" "The name of the service to move the robot")
(defconstant +service-name-get-base-origin+ "/suturo/get_base_origin" "The name of the service to get the base origin")
(defconstant +service-name-classify-objects+ "suturo/Classifier" "The name of the service to classify objects")
(defconstant +service-name-euroc-object-to-odom-combined+ "/suturo/euroc_object_to_odom_combined" "The name of the service to convert an EurocObject the a odom combined one")
(defconstant +service-name-add-collision-objects+ "/suturo/manipulation/add_collision_objects" "The nme of the service to add collision objects to the current scene")
(defconstant +waiting-time-before-scan+ 1)
  
(defun start-up-dependency-nodes()
  (call-service-state "start_manipulation")
  (call-service-state "start_perception")
  (call-service-state "start_classifier"))

(defun start-up-simulation()
  (setf-msg (value *taskdata*) (name) "task1_v1")
  (call-service-state "start_simulation"))

(defun plan ()
  (roslisp:with-ros-node (*name-node*)
    (cpl-impl:top-level
      (cpl-impl:par
        (state-init)
        (state-scan-map)
        (state-scan-shadow)
        (state-search-objects)
        (state-classify-objects)
        (state-pose-estimate-object)
        (loop while (not (eql (value *current-state*) :state-end)) do
          (cond  
            ((and (eql (value *current-state*) :state-search-objects) (eql (value *current-transition*) :transition-missing-objects)) (call-service-state "scan_obstacles"))
            ((and (eql (value *current-state*) :state-search-objects) (eql (value *current-transition*) :transition-no-objects-left)) (done))
            ((and (eql (value *current-state*) :state-scan-obstacles) (eql (value *current-transition*) :transition-map-scanned)) (call-service-state "scan_obstacles")) 
           ;; ((and (eql (value *current-state*) :state-scan-obstacles) (eql (value *current-transition*) :transition-new-image)) (call-service-state "classify_objects")) 
            ((and (eql (value *current-state*) :state-scan-obstacles) (eql (value *current-transition*) :transition-no-region-left)) (done)) 
            ((and (eql (value *current-state*) :state-classify-objects) (eql (value *current-transition*) :transition-objects-classified)) (call-service-state "focus_objects"))  
            ;((and (eql (value *current-state*) :state-focus-objects) (eql (value *current-transition*) :transition-focus-handle)) (call-service-state "pose_estimate_object")) 
            ;((and (eql (value *current-state*) :state-focus-objects) (eql (value *current-transition*) :transition-focus-object)) (call-service-state "pose_estimate_object")) 
            ((and (eql (value *current-state*) :state-pose-estimate-object) (eql (value *current-transition*) :transition-success)) (call-service-state "focus_objects")) 
            ((and (eql (value *current-state*) :state-pose-estimate-object) (eql (value *current-transition*) :transition-fail)) (call-service-state "focus_objects"))))))))

(defun call-create-taskdata ()
  (print "Calling create taskdata ")
  (if (not (roslisp:wait-for-service *name-service-create-taskdata* *timeout-service*))
           (progn 
             (print "Timed out")
             (setf (value *current-transition*) :transition-timed-out))
           (progn 
             (setf (value *taskdata*) (roslisp:msg-slot-value(roslisp:call-service *name-service-create-taskdata* 'suturo_interface_msgs-srv:StartPlanning) 'taskdata)))))

(defun call-service-state (service-name)
  (let
      ((full-service-name (concatenate 'string "suturo/state/" service-name)))
    (print (concatenate 'string "calling service: " service-name))
    (if (not (roslisp:wait-for-service full-service-name *timeout-service*))
        (let 
            ((timed-out-text (concatenate 'string "Timed out waiting for service " service-name)))
          (roslisp:ros-warn nil t timed-out-text)
          (setf (value *current-transition*) :transition-timed-out))
        (progn
          (setf (value *current-state*) (string-state-to-keyword service-name))
          (let 
              ((value (roslisp:call-service full-service-name 'suturo_interface_msgs-srv:TaskDataService :taskdata (value *taskdata*)))) 
            (setf (value *taskdata*) (roslisp:msg-slot-value value 'taskdata))
            (setf (value *current-transition*) (string-transition-to-keyword (roslisp:msg-slot-value value 'result))))))))

(defun string-transition-to-keyword(str)
  (cond
    ((string= str "start") :transition-start)
    ((string= str "successful") :transition-successful)
    ((string= str "mapScanned") :transition-map-scanned)
    ((string= str "success") :transition-success)
    ((string= str "missingObjects") :transition-missing-objects)
    ((string= str "noObjectsLeft") :transition-no-objects-left)
    ((string= str "mapScanned") :transition-map-scanned)
    ((string= str "newImage") :transition-new-image)
    ((string= str "noRegionLeft") :transition-no-region-left)
    ((string= str "objectsClassified") :transition-objects-classified)
    ((string= str "noObject") :transition-no-object)
    ((string= str "fail") :transition-fail)
    ((string= str "focusHandle") :transition-focus-handle)
    ((string= str "focusObject") :transition-focus-object)))

(defun string-state-to-keyword(str)
  (cond
    ((string= str "init") :state-init)
    ((string= str "scan_map") :state-scan-map)
    ((string= str "scan_shadow") :state-scan-shadow)
    ((string= str "search_objects") :state-search-objects)
    ((string= str "scan_obstacles") :state-scan-obstacles)
    ((string= str "classify_objects") :state-classify-objects)
    ((string= str "focus_objects") :state-focus-objects)
    ((string= str "pose_estimate_object") :state-pose-estimate-object)))

(def-cram-function state-init ()
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-init) (eql *current-transition* :transition-start)))
      (print "Executing state init ")
      (call-create-taskdata)
      (start-up-simulation) 
      (start-up-dependency-nodes) 
      (call-service-state "init") 
      (setf (value *current-transition*) :transition-successful)))

(defun done ()
  (format t "Done")
  (setf *current-state* "end"))
