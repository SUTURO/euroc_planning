(in-package :perception)

(defun call-gripper-service (options)
;;;  (with-ros-node ("two_ints_client")
    (call-service "suturo/GetGripper" 'suturo_perception_msgs-srv:GetGripper :s options))

(defun create-capability-string(&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  (let ((perception-capabilities "height,centroid,color"))
    (when cuboid
      (setf perception-capabilities (concatenate 'string perception-capabilities ",cuboid")))
    (when pose-estimation
      (setf perception-capabilities (concatenate 'string perception-capabilities ",ModelPoseEstimation"))
      (when object-ids
        (let ((s-ids "")) 
          (loop for id in object-ids do
                (setf s-ids (concatenate 'string s-ids (write-to-string id) "," )))
          (setf s-ids (concatenate 'string "(" (subseq s-ids 0 (-(length s-ids) 1)) ")" ))
          (setf perception-capabilities (concatenate 'string perception-capabilities s-ids)))))
    (return-from create-capability-string perception-capabilities)))

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
