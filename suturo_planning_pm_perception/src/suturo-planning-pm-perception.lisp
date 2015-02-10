(in-package :perception)


(defun get-gripper-perception (&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  "get the objects recognized by the gripper camera"
  (let (options)
  (setf options (create-capability-string cuboid pose-estimation object-ids))
  (call-gripper-service options)))

(defun call-gripper-service (options)
  "call the service suturo/GetGripper"
    (roslisp:call-service "suturo/GetGripper" 'suturo_perception_msgs-srv:GetGripper :s options))

(defun get-scene-perception (&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  "get the objects recognized by the scene camera"
  (let (options)
  (setf options (create-capability-string cuboid pose-estimation object-ids))
  (call-scene-service options)))

(defun call-scene-service (options)
  "call the service suturo/GetScene"
  (roslisp:call-service "suturo/GetScene" 'suturo_perception_msgs-srv:GetScene :s options))

(defun create-capability-string(&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  "Create the string that describes which capabilities are used by the perception"
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

(defun recognize-objects-of-interest(colors)
  "TODO:Write something meaningful"
  (let (color-message)
    (dolist(color colors)
      (setf color-message (list (roslisp:make-msg "std_msgs/ColorRGBA"
                                                  (r) (nth 0 color) (g) (nth 1 color) (b) (nth 2 color) (a) (nth 3 color)))))
      (roslisp:call-service "suturo/RecognizeOoI" 'suturo_perception_msgs-srv:RecognizeOoI :colors color-message)))

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

;;@INFO
;; (desig-prop-value mydesig 'color)
;; (defparameter mydesig (make-designator 'object `((color (1 2 3 4))) ))
(def-action-handler perceive (obj-designator)
  (print "ASDFFF"))

(def-action-handler perceive-scene-with (scenecam)
  (call-service-add-point-cloud scenecam))

(def-action-handler perceive-scene-with-origin (scenecam arm-origin)
  (call-service-add-point-cloud scenecam arm-origin))

(def-action-handler classify-object (obj-designator)
  "Implement me")

(def-action-handler focus-object (obj-designator)
  "Implement me")

(def-action-handler pose-estimate-object (obj-designator)
  "Implement me")

(defun call-service-add-point-cloud(scenecam &optional arm-origin)
  (if (not (roslisp:wait-for-service +service-name-add-point-cloud+ +timeout-service+))
      (progn
        (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-add-point-cloud+))
        (fail 'map-scanning-failed))
      (progn
        (if (not arm-origin)
            (roslisp:call-service +service-name-add-point-cloud+ 'suturo_interface_msgs-srv:AddPointCloud :scenecam scenecam)
            (roslisp:call-service +service-name-add-point-cloud+ 'suturo_interface_msgs-srv:AddPointCloud :scenecam scenecam :arm_origin arm-origin)))))


(cpm:def-process-module suturo-planning-pm-perception (desig)
  (apply #'call-action (reference desig)))
