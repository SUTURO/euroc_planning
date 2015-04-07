(in-package :manipulation)

(defvar *base-origin* (cpl:make-fluent :name :base-origin :value nil)"The current center of the base as geometry\_msgs:Point")

(defun init ()
  "Initializes the manipulation"
  (init-base-origin-subscriber))

(defun init-base-origin-subscriber()
  "Subscribes the base-origin-topic to get the center of the base"
  (roslisp:subscribe +base-origin-topic+ 'geometry_msgs-msg:PointStamped #'base-origin-cb))
 
(defun base-origin-cb (msg)
  "Callback for the base-origin-topic-subscriber"
  (setf (cpl:value *base-origin*) (roslisp:msg-slot-value msg 'point)))

; Helper functions
(defmethod call-action ((action-sym t) &rest params)
  "Standard implementation of call-action. Is called whenever an action couldn't be resolved"
  (roslisp:ros-info (suturo pm-manipulation)
                    "Unimplemented operation `~a' with parameters ~a. Doing nothing."
                    action-sym params)
  (sleep 0.5))

(defgeneric call-action (action &rest params)
  (:documentation "Generic method to define an interface for executing actions. Whenever the function perform '(perform my-action-designator)' is executed, the prolog-reasoning-engine
 	 	 		matches the designator-properties of my-action-designator with conditions defined in the file designators.lisp and decides which action is going to be performed.
 	 	 		* Arguments
 	 	 		- action :: 
 	 	 		- params :: The parameter of the actions that should be called.
 	 	 		"
))

(defmethod call-action :around (action-sym &rest params)
  "Standard implementation of call-action. Is called whenever an action could be resolved"
  (roslisp:ros-info
    (suturo pm-manipulation)
    "Executing action `~a' with parameters ~a..."
    action-sym params)
  (prog1
    (call-next-method)
    (roslisp:ros-info (suturo pm-manipulation)
                      "Done executing action `~a'."
                      action-sym params)))

(defmacro def-action-handler (name args &body body)
  "
  * Arguments
  - name :: The name of the function
  - args :: The arguments of the function
  - body :: The body of the function
  * Description
  Defines a macro to create specific implementations of the generic function 'call-action'. Use this macro to define your actions !
  "
  (alexandria:with-gensyms (action-sym params)
    (if (not (typep (first body) 'string))
        `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
                                       (destructuring-bind ,args ,params ,@body))
        `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
                                       ,(first body)
                                       (destructuring-bind ,args ,params ,@body)))))

; To see how these action handlers are implemented for the pr2, see
; https://github.com/cram-code/cram\_pr2/blob/master/pr2_manipulation_process_module/src/action-handlers.lisp

(def-action-handler no-navigation (goal)
  "Do nothing"
  nil)

(def-action-handler navigation (goal &optional (do-not-blow-up-list #()))
  "
  Move the robot to the given goal position
  * Arguments

  - goal :: The pose wherer the robot should be moved to - location-designator || geometriy\_msgs/PoseStamped
  - do-not-blow-up-list :: A list of object-names which should not be blown up during planning - (list string)
  - Return :: True if the robot has been moved to the given pose false otherwise - bool
  "
  (print "goal in navigation")
  (format t "~a" goal)
  (let ((goal1 (if (typep goal 'location-designator) (cl-tf:pose-stamped->msg (reference goal)) goal)))
    (format t "~a" goal1)
    (if (not (roslisp:wait-for-service +service-name-move-mastcam+ +timeout-service+))
        (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-move-mastcam+)))
          (roslisp:ros-warn nil t timed-out-text))
        (progn
          (let ((response nil))
            (if do-not-blow-up-list
                (setf response (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move
                                                     :type (roslisp-msg-protocol:symbol-code 'suturo_manipulation_msgs-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                                     :goal_pose goal1
                                                     :do_not_blow_up_list do-not-blow-up-list))
                (setf response (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move
                                                     :type (roslisp-msg-protocol:symbol-code 'suturo_manipulation_msgs-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                                     :goal_pose goal1)))
            (if (not (msg-slot-value response 'result))
                (cpl-impl:fail 'manipulation-failure))
            response)))))

(def-action-handler follow (pose)
  "Follow head with pose."
  ; Will not be implemented as we don't have a head to follow an object
)

(def-action-handler park (obj)
  "Moves the arms to a park position"
  ; Will not be implemented as we don't have a parking position
)

(def-action-handler lift (collision-object grasp-point)
  "Lifts an arm by a distance") 

(def-action-handler grasp (object-designator)
  "
  Grasp the given object and lift it
  * Arguments
  - object-designator :: The designator describing the object - object-designator
  "
  (let ((collision-object (desig-prop-value object-designator 'cram-designator-properties:collision-object)))
    (if (not (roslisp:wait-for-service +service-name-grasp-object+ +timeout-service+))
        (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-grasp-object+)))
          (roslisp:ros-warn nil t timed-out-text)
          (cpl-impl:fail 'cram-plan-failures:manipulation-failure))
        (progn
          (let* ((response (roslisp:call-service +service-name-grasp-object+ 'suturo_manipulation_msgs-srv:GraspObject
                                                :object (roslisp:setf-msg collision-object (stamp header) (roslisp:ros-time))
                                                :density (get-object-density collision-object (roslisp:msg-slot-value environment:*yaml* 'objects))))
                 (result (roslisp:msg-slot-value response 'result))
                 (grasp-position (roslisp:msg-slot-value response 'grasp_position)))
            (if (not result)
                (cpl-impl:fail 'cram-plan-failures:manipulation-failure))
            (let ((new-desig (copy-designator object-designator :new-description `((grasp-position ,grasp-position))))) 
              (equate object-designator new-desig)))))))

(defun get-object-density (collision-object objects)
  "
  Get the density of the given object from the given list of all objects
  * Arguments
  - collision-object :: The object to get the density of - moveit\_msgs/CollisionObject
  - objects :: The list of objects to get the density from
  - Return :: The density of the given object - int
  "
  (let ((result nil))
    (loop for object across objects do
      (if (string= (roslisp:msg-slot-value object 'name) (roslisp:msg-slot-value collision-object 'id))
          (setf result (elt (roslisp:msg-slot-value object 'primitive_densities) 0))))
    result))


(def-action-handler carry (obj-designator)
  "Carries the object"
  ; Will not be implemented as we have nothing to do within this action
)

(def-action-handler put-down (collision-object location grasp)
  "
  Put the given object down at the given location.
  * Arguments
  - collision-object :: The object which should be placed - moveit\_msgs/CollisionObject
  - location :: The pose where the object should be placed - cl-tf::pose-stamped
  - grasp :: The pose where the given object has been grasped - geometry\_msgs/PoseStamped
  "
  (if (not (roslisp:wait-for-service +service-name-move-robot+ +timeout-service+))
    (let ((timed-out-text (concatenate 'string "Timed out waiting for service" +service-name-move-robot+)))
      (roslisp:ros-warn nil t timed-out-text))
    (progn
			(let ((location-msg (msg-slot-value (msg-slot-value (cl-tf:pose-stamped->msg location) 'pose) 'position)))
				(with-fields (id) collision-object
          (let ((result nil)
                (place-block (gensym "BLOCK"))
                (try-block (gensym "BLOCK"))
                (dist-to-obj (cl-transforms:v-norm (cl-transforms:v- (msg->vector (msg-slot-value (msg-slot-value grasp 'pose) 'position)) (msg->vector (msg-slot-value (get-fingertip grasp) 'point))))))
            (ros-info (achieve put-down) "Distance to object: ~a" dist-to-obj)
            (block place-block
              (let ((place-poses (get-place-positions collision-object location-msg dist-to-obj grasp)))
              (loop for place-pose in place-poses do
                (block try-BLOCK
                  ; Move to the pre place position
                  (ros-info (achieve put-down) "Move to pre place position")
                  (let ((response (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move
                                                        :type (roslisp-msg-protocol:symbol-code 'suturo_manipulation_msgs-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                                        :goal_pose (get-pre-place-position place-pose)
                                                        :do_not_blow_up_list `(,id))))
                    (with-fields (result) response
                      (if (not result)
                        (return-from try-BLOCK))))
                  ; Place the object
                  (ros-info (achieve put-down) "Place the object")
                  (let ((response (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move
                                                        :type (roslisp-msg-protocol:symbol-code 'suturo_manipulation_msgs-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                                        :goal_pose place-pose
                                                        :do_not_blow_up_list `(,id "map"))))
                    (with-fields (result) response
                      (if (not result)
                        (return-from try-BLOCK))))
                  ; Open the gripper
                  (ros-info (achieve put-down) "Open the Gripper")
                  (if (not (roslisp:wait-for-service +service-name-open-gripper+ +timeout-service+))
                      (let ((timed-out-text (concatenate 'string "Timed out waiting for service" +service-name-open-gripper+)))
                        (roslisp:ros-warn nil t timed-out-text))
                     (let ((response (roslisp:call-service +service-name-open-gripper+ 'suturo_manipulation_msgs-srv:OpenGripper)))
                        (with-fields (result) response
                          (if (not result)
                            (cram-language-implementation:fail 'cram-plan-failures:manipulation-failure)))))
                  ; Move to pre grasp
                  (ros-info (achieve put-down) "Move to pre grasp")
                  (if (not (msg-slot-value (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move
                                                        :type (roslisp-msg-protocol:symbol-code 'suturo_manipulation_msgs-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                                        :goal_pose (get-pre-grasp place-pose)
                                                        :do_not_blow_up_list `(,id "map")) 'result))
                      (progn
                        ; Move to pre place
                        (ros-info (achieve put-down) "Move to pre place")
                        (roslisp:call-service +service-name-move-robot+ 'suturo_manipulation_msgs-srv:Move
                                              :type (roslisp-msg-protocol:symbol-code 'suturo_manipulation_msgs-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                              :goal_pose (get-pre-place-position place-pose)
                                              :do_not_blow_up_list `(,id "map"))))
                  (setf result T)
                  (return-from place-BLOCK)))))
            result))))))


;;----------service calls ----------------------------
(defun call-add-collision-objects(objects)
  "* Arguments 
- objects :: The objects as suturo\_perception\_msgs-msg:EurocObject that should be added to the collision scene
* Return Value
Ignored
* Description
Adds the given objects as collosion-objects to the moveit environment"
  (if (not (roslisp:wait-for-service +service-name-add-collision-objects+ +timeout-service+))
        (print "Timed out")
        (roslisp:call-service +service-name-add-collision-objects+ 'suturo_manipulation_msgs-srv:AddCollisionObjects :objects objects)))

(cpm:def-process-module suturo-planning-pm-manipulation (desig)
  (apply #'call-action (reference desig)))


