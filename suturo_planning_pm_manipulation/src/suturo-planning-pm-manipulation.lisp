(in-package :manipulation)

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
  "Moves the robot to the goal position")

(def-action-handler follow (pose)
  "Follow head with pose."
  ; TODO: Implement me
  )

(def-action-handler park (obj)
  "Moves the arms to a park position"
  ; TODO: Implement me
  )

(def-action-handler lift (obj-designator)
  "Lifts an arm by a distance"
  ; TODO: Implement me
  )

(def-action-handler grasp (obj-designator)
  "Grasps the object specified by the obj-designator"
  ; TODO: Implement me
  )

(def-action-handler carry (obj-designator)
  "Carries the object"
  ;TODO: Implement me
  )

(def-action-handler put-down (obj-designator location)
  "Puts the object specified by the obj-designator down at a location"
  ; TODO: Implement me
  )

(cpm:def-process-module suturo-planning-pm-manipulation (desig)
  (apply #'call-action (reference desig)))
