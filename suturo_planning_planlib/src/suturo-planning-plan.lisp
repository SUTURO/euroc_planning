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
:state-clean-up-plan
:state-choose-object
:state-grasp-object
:state-place-object
:state-check-placement
:state-end
:state-fail


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
:transition-object-chosen
:transition-retry
:transition-object-not-in-planning-scene
:transition-no-grasp-position
:transition-no-object-attached
:transition-no-place-position
:transition-on-target
:transition-not-on-target


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

(defconstant +service-name-classify-objects+ "suturo/Classifier" "The name of the service to classify objects")
(defconstant +service-name-euroc-object-to-odom-combined+ "/suturo/euroc_object_to_odom_combined" "The name of the service to convert an EurocObject the a odom combined one")
(defconstant +service-name-get-collision-object+ "/suturo/manipulation/get_collision_object" "The name of the service to get a collision object of the planning scene")
(defconstant +service-name-add-collision-objects+ "/suturo/manipulation/add_collision_objects" "The name of the service to add collision objects to the current scene")
(defconstant +service-name-mark-region-as-object-under-point+ "/suturo/mark_region_as_object_under_point" "The name of the service to mark a region as object")
(defconstant +service-name-current-map-to-collision-object+ "/suturo/current_map_to_collision_object")

(defun start-up-dependency-nodes()
  (call-service-state "start_manipulation")
  (call-service-state "start_perception")
  (call-service-state "start_classifier"))

(defun start-up-simulation(task_name)
  (setf-msg (value *taskdata*) (name) task_name)
  (call-service-state "start_simulation"))

(defun do-planning (task_name)
      (cpl-impl:par
        (state-init task_name)
        (state-search-objects)
        (state-classify-objects)
        (state-pose-estimate-object)
        (state-focus-objects)
        (loop while (not (or (eql (value *current-state*) :state-end) (eql (value *current-state*) :state-fail))) do
          (cond  
            ((and (eql (value *current-state*) :state-search-objects) (eql (value *current-transition*) :transition-missing-objects)) (call-service-state "scan_obstacles"))
            ((and (eql (value *current-state*) :state-search-objects) (eql (value *current-transition*) :transition-no-objects-left)) (call-service-state "clean_up_plan"))
            ((and (eql (value *current-state*) :state-scan-obstacles) (eql (value *current-transition*) :transition-map-scanned)) (call-service-state "scan_obstacles")) 
            ((and (eql (value *current-state*) :state-scan-obstacles) (eql (value *current-transition*) :transition-no-region-left)) (call-service-state "clean_up_plan")) 
            ((and (eql (value *current-state*) :state-clean-up-plan) (eql (value *current-transition*) :transition-success)) (call-service-state "choose_object"))
            ((and (eql (value *current-state*) :state-clean-up-plan) (eql (value *current-transition*) :transition-fail)) (failed))
            ((and (eql (value *current-state*) :state-choose-object) (eql (value *current-transition*) :transition-object-chosen)) (call-service-state "grasp_object"))
            ((and (eql (value *current-state*) :state-choose-object) (eql (value *current-transition*) :transition-success)) (done))
            ((and (eql (value *current-state*) :state-choose-object) (eql (value *current-transition*) :transition-retry)) (call-service-state "clean_up_plan"))
            ((and (eql (value *current-state*) :state-choose-object) (eql (value *current-transition*) :transition-no-objects-left)) (done))
            ((and (eql (value *current-state*) :state-grasp-object) (eql (value *current-transition*) :transition-success)) (call-service-state "place_object"))
            ((and (eql (value *current-state*) :state-grasp-object) (eql (value *current-transition*) :transition-object-not-in-planning-scene)) (failed))
            ((and (eql (value *current-state*) :state-grasp-object) (eql (value *current-transition*) :transition-no-grasp-position)) (failed))
            ((and (eql (value *current-state*) :state-grasp-object) (eql (value *current-transition*) :transition-fail)) (failed))
            ((and (eql (value *current-state*) :state-place-object) (eql (value *current-transition*) :transition-success)) (call-service-state "check_placement"))
            ((and (eql (value *current-state*) :state-place-object) (eql (value *current-transition*) :transition-fail)) (failed))
            ((and (eql (value *current-state*) :state-place-object) (eql (value *current-transition*) :transition-no-object-attached)) (call-service-state "grasp_object"))
            ((and (eql (value *current-state*) :state-place-object) (eql (value *current-transition*) :transition-no-place-position)) (call-service-state "place_object"))
            ((eql (value *current-state*) :state-check-placement) (call-service-state "choose_object"))))))

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
          (setf (value *current-transition*) :transition-nil)
          (setf (value *current-state*) (string-state-to-keyword service-name))
          (let 
              ((value (roslisp:call-service full-service-name 'suturo_interface_msgs-srv:TaskDataService :taskdata (value *taskdata*)))) 
            (setf (value *taskdata*) (roslisp:msg-slot-value value 'taskdata))
            (setf (value *current-transition*) (string-transition-to-keyword (roslisp:msg-slot-value value 'result))))))))


(defun call-add-collision-objects(objects)
  (print "Calling add collision objects")
  (if (not (roslisp:wait-for-service +service-name-add-collision-objects+ *timeout-service*))
      (progn 
        (print "Timed out")
        (setf (value *current-transition*) :transition-timed-out)) 
      (roslisp:call-service +service-name-add-collision-objects+ 'suturo_planning_manipulation-srv:AddCollisionObjects :objects objects)))

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
    ((string= str "focusObject") :transition-focus-object)
    ((string= str "objectChosen") :transition-object-chosen)
    ((string= str "retry") :transition-retry)
    ((string= str "objectNotInPlanningscene") :transition-object-not-in-planning-scene)
    ((string= str "noGraspPosition") :transition-no-grasp-position)
    ((string= str "noObjectAttached") :transition-no-object-attached)
    ((string= str "noPlacePosition") :transition-no-place-position)
    ((string= str "onTarget") :transition-on-target)
    ((string= str "notOnTarget") :transition-not-on-target)))

(defun string-state-to-keyword(str)
  (cond
    ((string= str "init") :state-init)
    ((string= str "scan_map") :state-scan-map)
    ((string= str "scan_shadow") :state-scan-shadow)
    ((string= str "search_objects") :state-search-objects)
    ((string= str "scan_obstacles") :state-scan-obstacles)
    ((string= str "classify_objects") :state-classify-objects)
    ((string= str "focus_objects") :state-focus-objects)
    ((string= str "pose_estimate_object") :state-pose-estimate-object)
    ((string= str "clean_up_plan") :state-clean-up-plan)
    ((string= str "choose_object") :state-choose-object)
    ((string= str "grasp_object") :state-grasp-object)
    ((string= str "place_object") :state-place-object)
    ((string= str "check_placement") :state-check-placement)
    ((string= str "end") :state-end)
    ((string= str "fail") :state-fail)))

(def-cram-function state-init (task_name)
    (loop while T do
      (cpl-impl:wait-for (fl-and (eql *current-state* :state-init) (eql *current-transition* :transition-start)))
      (print "Executing state init ")
      (setf (value *current-transition*) :transition-nil)
      (call-create-taskdata)
      (start-up-simulation task_name) 
      (start-up-dependency-nodes) 
      (call-service-state "init") 
      (manipulation:init)
      (setf (value *current-transition*) :transition-successful)))

(defun done ()
  (format t "Done")
  (setf (value *current-state*) :state-end))

(defun failed ()
  (format t "FAILED!")
  (setf (value *current-state*) :state-fail))
