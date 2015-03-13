(in-package :manipulation)

(defvar *base-origin* (cpl:make-fluent :name :base-origin :value nil)"The current center of the base as geometry_msgs:Point")

(defun init ()
  (init-base-origin-subscriber))

(defun init-base-origin-subscriber()
  (roslisp:subscribe +base-origin-topic+ 'geometry_msgs-msg:PointStamped #'base-origin-cb))
 
(defun base-origin-cb (msg)
  (setf (cpl:value *base-origin*) (roslisp:msg-slot-value msg 'point)))

; Helper functions
(defgeneric call-action (action &rest params))

(defmethod call-action ((action-sym t) &rest params)
  (roslisp:ros-info (suturo pm-manipulation)
                    "Unimplemented operation `~a' with parameters ~a. Doing nothing."
                    action-sym params)
  (sleep 0.5))

(defmethod call-action :around (action-sym &rest params)
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
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
      (destructuring-bind ,args ,params ,@body))))

; To see how these action handlers are implemented for the pr2, see
; https://github.com/cram-code/cram_pr2/blob/master/pr2_manipulation_process_module/src/action-handlers.lisp

(def-action-handler no-navigation (goal)
  "Do nothing"
  nil)

(def-action-handler navigation (goal &optional (do-not-blow-up-list #()))
  "Moves the robot to the goal position"
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
                (fail 'manipulation-failure))
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


;(def-action-handler grasp (object-designator)
;  "Grasps the object specified by the obj-designator"
;  (let ((collision-object (desig-prop-value object-designator 'collision-object)))
;  (if (not (roslisp:wait-for-service +service-name-close-gripper+ +timeout-service+))
;    (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-close-gripper+)))
;      (roslisp:ros-warn nil t timed-out-text))
;    (progn
;        (with-fields (result joint_state) (roslisp:call-service +service-name-close-gripper+ 'suturo_manipulation_msgs-srv:CloseGripper
;                                                                :object collision-object)
;          (if (not result)
;              (fail 'manipulation-failure)
;              (with-fields (position) joint_state
;                (make-designator 'action (update-designator-properties `((grasp-point position)) (description object-designator))))))))))

;(def-action-handler grasp (object-designator)
;  (defparameter my-obj-designator object-designator)
;  "Grasps the object specified by the obj-designator"
;  (let ((collision-object (desig-prop-value object-designator 'cram-designator-properties:collision-object)))
;    (if (not (roslisp:wait-for-service +service-name-grasp-object+ +timeout-service+))
;        (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-grasp-object+)))
;          (roslisp:ros-warn nil t timed-out-text))
;        (progn
;          (let* ((response (roslisp:call-service +service-name-grasp-object+ 'suturo_interface_msgs-srv:GraspObject
;                                                :object (roslisp:setf-msg collision-object (stamp header) (roslisp:ros-time))
;                                                :density (get-object-density collision-object (roslisp:msg-slot-value environment:*yaml* 'objects))))
;                 (result (roslisp:msg-slot-value response 'result))
;                 (grasp-position (roslisp:msg-slot-value response 'grasp_position)))
;            (if (not result)
;                (fail 'manipulation-failure))
;            (let ((new-desig (copy-designator object-designator :new-description `((grasp-position ,grasp-position))))) 
;              (equate object-designator new-desig)))))))

(defun get-object-density (collision-object objects)
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
  "Puts the object specified by the obj-designator down at a location"
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
                (setf *place-pose* place-pose)
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
  (print "Calling add collision objects")
  (if (not (roslisp:wait-for-service +service-name-add-collision-objects+ +timeout-service+))
        (print "Timed out")
        (roslisp:call-service +service-name-add-collision-objects+ 'suturo_manipulation_msgs-srv:AddCollisionObjects :objects objects)))

(cpm:def-process-module suturo-planning-pm-manipulation (desig)
  (apply #'call-action (reference desig)))
