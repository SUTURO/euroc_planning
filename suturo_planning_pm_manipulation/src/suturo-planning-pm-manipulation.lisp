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

(defmethod call-ros-service (service-name &rest args)
  (if (not (roslisp:wait-for-service service-name +timeout-service+))
    (let ((timed-out-text (concatenate 'string "Times out waiting for service" service-name)))
      (roslisp:ros-warn nil t timed-out-text))
    (progn
      (roslisp:call-service service-name args)))
)

; To see how these action handlers are implemented for the pr2, see
; https://github.com/cram-code/cram_pr2/blob/master/pr2_manipulation_process_module/src/action-handlers.lisp

(def-action-handler navigation (goal)
  "Moves the robot to the goal position")

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
  (let ((position (make-msg "geometry_msgs/Point" :x (first grasp-point)
                                                  :y (second grasp-point)
                                                  :z (third grasp-point))))
    (let ((request (make-request 'suturo_planning_manipulation-srv:Move
                    (roslisp-msg-protocol:symbol-code 'suturo_planning_manipulation-srv:Move-Request :ACTION_MOVE_ARM_TO)
                    position
                    collision-object-name))
      )
      (let ((response (call-ros-service request)))
        (if (not (msg-slot-value response 'result))
          (fail 'manipulation-failure)
        )
      )
    )
  ) 
)

(def-action-handler grasp (object-designator)
  "Grasps the object specified by the obj-designator"
  (with-desig-props (collision-object) object-designator
    (let ((request (roslisp:make-request 'suturo_planning_manipulation-srv:CloseGripper collision-object nil)))
      (let ((response (call-ros-service +service-name-close-gripper+ request)))
        (with-fields (result joint_state) response
          (if (not result)
            (fail 'manipulation-failure)
            (with-fields (position) joint_state
              (make-designator 'action (update-designator-properties `((grasp-point (position))) (description object-designator))))
          )
        )
      )
    )
  )
)

(def-action-handler carry (obj-designator)
  "Carries the object"
  ; Will not be implemented as we have nothing to do within this action
)

(def-action-handler put-down (obj-designator location)
  "Puts the object specified by the obj-designator down at a location"
  ; TODO: Implement me
  )

(cpm:def-process-module suturo-planning-pm-manipulation (desig)
  (apply #'call-action (reference desig)))
