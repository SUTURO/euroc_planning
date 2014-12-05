(in-package :perception)

(defgeneric call-action (action &rest params))

(defmethod call-action ((action-sym t) &rest params)
  (roslisp:ros-info (suturo pm-perception)
                    "Unimplemented operation `~a' with parameters ~a. Doing nothing."
                    action-sym params)
  (sleep 0.5))

(defmethod call-action :around (action-sym &rest params)
  (roslisp:ros-info
    (suturo pm-perception)
    "Executing action `~a' with parameters ~a..."
    action-sym params)
  (prog1
    (call-next-method)
    (roslisp:ros-info (suturo pm-perception)
                      "Done executing action `~a'."
                      action-sym params)))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
      (destructuring-bind ,args ,params ,@body))))

(def-action-handler perceive (obj-designator)
  "Returns a list of objects that can be perceived without
   moving and match the object-designator"
  ; TODO: Implement me
  (roslisp:ros-info (action perceive) "Returning object.")
  (list obj-designator))

(cpm:def-process-module suturo-planning-pm-perception (desig)
  (apply #'call-action (reference desig)))
