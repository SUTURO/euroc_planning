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

(def-action-handler navigation (goal)
  "Moves the robot to the goal position"
  (if (not (roslisp:wait-for-service  +service-name-move-robot+ +timeout-service+))
    (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-move-robot+)))
      (roslisp:ros-warn nil t timed-out-text))
    (progn
      (let ((response (roslisp:call-service +service-name-move-robot+ 'suturo_planning_manipulation-srv:Move
                                        :type (roslisp-msg-protocol:symbol-code 'suturo_planning_manipulation-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                        :goal_pose goal)))
        (if (not (msg-slot-value response 'result))
          (fail 'manipulation-failure))))))

(def-action-handler follow (pose)
  "Follow head with pose."
  ; Will not be implemented as we don't have a head to follow an object
)

(def-action-handler park (obj)
  "Moves the arms to a park position"
  ; Will not be implemented as we don't have a parking position
)

(def-action-handler lift (grasp-point collision-object-name)
  "Lifts an arm by a distance"
  (if (not (roslisp:wait-for-service +service-name-move-robot+ +timeout-service+))
    (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-move-robot+)))
      (roslisp:ros-warn nil t timed-out-text))
    (progn
      (let ((position (make-msg "geometry_msgs/Pose"
                                :position (make-msg "geometry_msgs/Point"
                                                    :x (first grasp-point)
                                                    :y (second grasp-point)
                                                    :z (third grasp-point)))))
        (let ((response (roslisp:call-service +service-name-move-robot+ 'suturo_planning_manipulation-srv:Move
                                          :type (roslisp-msg-protocol:symbol-code 'suturo_planning_manipulation-srv:Move-Request :ACTION_MOVE_ARM_TO)
                                          :goal_pose position
                                          :do_not_blow_up_list=collision-object-name)))
          (if (not (msg-slot-value response 'result))
            (fail 'manipulation-failure))))))) 


(def-action-handler grasp (object-designator)
  "Grasps the object specified by the obj-designator"
  (if (not (roslisp:wait-for-service +service-name-close-gripper+ +timeout-service+))
    (let ((timed-out-text (concatenate 'string "Times out waiting for service" +service-name-close-gripper+)))
      (roslisp:ros-warn nil t timed-out-text))
    (progn
      (with-desig-props (collision-object) obj-designator
        (let ((response (roslisp:call-service +service-name-close-gripper+ 'suturo_planning_manipulation-srv:CloseGripper collision-object)))
          (with-fields (result joint_state) response
            (if (not result)
              (fail 'manipulation-failure)
              (with-fields (position) joint_state
                (make-designator 'action (update-designator-properties `((grasp-point (position))) (description object-designator)))))))))))


(def-action-handler carry (obj-designator)
  "Carries the object"
  ; Will not be implemented as we have nothing to do within this action
)

(def-action-handler put-down (obj-designator location)
  "Puts the object specified by the obj-designator down at a location"
)

;;----------service calls ----------------------------
(defun call-add-collision-objects(objects)
  (print "Calling add collision objects")
  (if (not (roslisp:wait-for-service +service-name-add-collision-objects+ +timeout-service+))
        (print "Timed out")
        (roslisp:call-service +service-name-add-collision-objects+ 'suturo_planning_manipulation-srv:AddCollisionObjects :objects objects)))

(cpm:def-process-module suturo-planning-pm-manipulation (desig)
  (apply #'call-action (reference desig)))
