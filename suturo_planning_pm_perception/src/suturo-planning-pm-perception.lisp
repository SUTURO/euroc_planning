(in-package :perception)

"* Actions

"

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    (defparameter *body* body)
    (if (not (typep (first body) 'string))
        `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
                                       (destructuring-bind ,args ,params ,@body))
        `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
                                       ,(first body)
                                       (destructuring-bind ,args ,params ,@body)))))
        

;-------------------- Low level perception ------------------------------

(def-action-handler get-gripper-perception (&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  "
* Arguments
- cuboid ::  
- pose-estimation :: Determines wether perception pipeline should estimate the pose of the objects
- object-ids :: Ids of the objects
* Description
Get the objects recognized by the gripper camera
"
  (let (options)
  (setf options (create-capability-string cuboid pose-estimation object-ids))
  (call-gripper-service options)))

(defun call-gripper-service (options)
  "
* Arguments
- options :: The options that the service call transmits to the receiver
* Description
Call the service suturo/GetGripper
"
    (roslisp:call-service "suturo/GetGripper" 'suturo_perception_msgs-srv:GetCameraPerception :s options))

(defun get-scene-perception (&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  "
* Arguments
- cuboid ::
- pose-estimation :: Determines wether perception pipeline should estimate the pose of the objects
- object-ids :: Ids of the objects
* Description
Get the objects recognized by the scene camera
"
  (let (options)
  (setf options (create-capability-string cuboid pose-estimation object-ids))
  (call-scene-service options)))

(defun call-scene-service (options)
  "
* Arguments
- options :: The options that the service call transmits to the receiver
* Description
Calls the service suturo/GetScene
"
  (roslisp:call-service "suturo/GetScene" 'suturo_perception_msgs-srv:GetCameraPerception :s options))

(defun create-capability-string(&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  "
* Arguments
- cuboid :: 
- pose-estimation :: Determines wether perception pipeline should estimate the pose of the objects
- object-ids :: Ids of the objects
* Description
Create the string that describes which capabilities are used by the perception
"
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
  "
* Arguments
- colors :: Colors of the objects
* Description
Recognizes Objects of Interest. It gets a list of colors and returns a list of objects that matches these colors.
"
  (let (color-message)
    (dolist(color colors)
      (setf color-message (list (roslisp:make-msg "std_msgs/ColorRGBA"
                                                  (r) (nth 0 color) (g) (nth 1 color) (b) (nth 2 color) (a) (nth 3 color)))))
      (roslisp:call-service "suturo/RecognizeOoI" 'suturo_perception_msgs-srv:RecognizeOoI :colors color-message)))

  "
* Arguments
- action ::
- params ::
* Description

"
(defgeneric call-action (action &rest params))
 
 "
* Arguments
- action-sym ::
- params ::
- params
* Description

"
(defmethod call-action ((action-sym t) &rest params)
  (roslisp:ros-info (suturo pm-perception)
                    "Unimplemented operation `~a' with parameters ~a. Doing nothing."
                    action-sym params)
  (sleep 0.5))

  "
* Arguments
- action-sym ::
- params :: 
* Description

"
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

;;@INFO
;; (desig-prop-value mydesig 'color)
;; (defparameter mydesig (make-designator 'object `((color (1 2 3 4))) ))
  "
* Arguments
- obj-designator::
* Description

"
(def-action-handler perceive (obj-designator)
  (list obj-designator))

;---------------scan map -------------------------------------
  "
* Arguments
- scenecam :: the camera the map should be scanned with
* Description
Scans the map and add the perceived point cloud to the map.
"
(def-action-handler perceive-scene-with (scenecam)
  "Scans the whole map
* Arguments
- scenecam :: blub
"
  (cram-beliefstate:add-topic-image-to-active-node "/euroc_interface_node/cameras/tcp_rgb_cam")
  (call-service-add-point-cloud scenecam))

  "
* Arguments
- scenecam :: the camera the map should be scanned with
- arm-origin :: the origin of the arm
* Description
Scans the map and add the perceived point cloud to the map.
"
(def-action-handler perceive-scene-with-origin (scenecam arm-origin)
  (cram-beliefstate:add-topic-image-to-active-node "/euroc_interface_node/cameras/scene_rgb_cam")
  (call-service-add-point-cloud scenecam arm-origin))

;----------------pose estimation------------------------------
  "
* Arguments
- ids :: 
* Description
Estimates the pose of the objects. Returns a list of EurocObjects.
"
(def-action-handler pose-estimate-object (ids)
      (let ((pose-estimated-object (elt (roslisp:msg-slot-value (call-gripper-service (create-capability-string nil T ids)) 'objects) 0)))
      ;(let ((pose-estimated-object (elt (roslisp:msg-slot-value (get-gripper-perception nil T id) 'objects) 0))) ;;ID as vector/array ? 
        (if (not pose-estimated-object)
            (print "Pose estimation failed, no object returned")
            (progn
              (setf pose-estimated-object (roslisp:msg-slot-value (call-euroc-object-to-odom-combined pose-estimated-object) 'converted))
              (if (or (not pose-estimated-object) (not (roslisp:msg-slot-value pose-estimated-object 'mpe_success)))
                  (print "Pose estimation failed, couldn't pose estimate object")
                  (progn
                    (manipulation:call-add-collision-objects (vector (roslisp:msg-slot-value pose-estimated-object 'mpe_object)))
                    ))))
      pose-estimated-object))
      

;;-------------------classify--------------------------------
  "
* Arguments
- perceived-object :: the Eurocobject that should be classified.
* Description
Classifies an EurocObject. Returns the classified EurocObject

"
(def-action-handler classify-object (perceived-object)
  ;cuboid = TRUE, pose-estimation = false, objects-ids = []
  (analyze-perceived-object perceived-object))

  "
* Arguments
- object :: Euroc object that should be analyzed
* Description

"
(defun analyze-perceived-object(object)
    (let ((matched-object (roslisp:msg-slot-value (call-classify-object object) 'classifiedObject))
          (type-obstacle (roslisp:symbol-code 'suturo_perception_msgs-msg:EurocObject :OBSTACLE))
          (type-unknown (roslisp:symbol-code 'suturo_perception_msgs-msg:EurocObject :UNKNOWN))
          (type-table (roslisp:symbol-code 'suturo_perception_msgs-msg:EurocObject :TABLE))
          (type-object (roslisp:symbol-code 'suturo_perception_msgs-msg:EurocObject :OBJECT)))
      (cond 
        ((= (roslisp:msg-slot-value matched-object 'c_type) type-obstacle) (handle-object-obstacle matched-object))
        ((or (= (roslisp:msg-slot-value matched-object 'c_type) type-unknown) (= (roslisp:msg-slot-value matched-object 'c_type) type-table)) 
         (handle-object-unknown-or-table matched-object))
        ((= (roslisp:msg-slot-value matched-object 'c_type) type-object) (handle-object matched-object)))))

  "
* Arguments
- matched-object :: 
* Description

"
(defun handle-object-obstacle (matched-object)
  (if (roslisp:msg-slot-value matched-object 'c_cuboid_success)
      (print "Found obstacle")))

  "
* Arguments
- matched-object :: 
* Description
Handles the behaviour when an unknown object or the table is perceived
"
(defun handle-object-unknown-or-table (matched-object) 
  (if (roslisp:msg-slot-value matched-object 'c_cuboid_success)
      (print "Unknown object or table perceived. Ignoring it for now.")))

  "
* Arguments
- matched-object :: the found object
* Description
Gets called when a valid object is found.
"
(defun handle-object (matched-object)
  (print "Found object")
  (roslisp:msg-slot-value (call-euroc-object-to-odom-combined matched-object)'converted))

;;--------------Service calls ----------------------
  "
* Arguments
- object :: the object that should be classified
* Description
Calls the service that classifies the object
"
(defun call-classify-object (object)
  (print "Calling classify object ")
  (if (not (roslisp:wait-for-service +service-name-classify-objects+ +timeout-service+))
        (print "Timed out")
        (roslisp:call-service +service-name-classify-objects+ 'suturo_perception_msgs-srv:Classifier :unclassifiedObject object)))
  "
* Arguments
- object :: the EurocObject that should be converted
* Description
Transform the EurocObject to /odom_combined . Also returns an EurocObject.
"
(defun call-euroc-object-to-odom-combined(object)
  (print "Calling euroc object to odom combined ")
  (if (not (roslisp:wait-for-service +service-name-euroc-object-to-odom-combined+ +timeout-service+))
      (print "Timed out")
      (roslisp:call-service +service-name-euroc-object-to-odom-combined+ 'suturo_environment_msgs-srv:EurocObjectToOdomCombined :toConvert object)))
  "
* Arguments
- scenecam :: the camera
- arm-origin :: the origin of the arm
* Description
Adds the Point Cloud from the camera to the map. 
"
(defun call-service-add-point-cloud(scenecam &optional arm-origin)
  (if (not (roslisp:wait-for-service +service-name-add-point-cloud+ +timeout-service+))
      (progn
        (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-add-point-cloud+))
        (fail 'map-scanning-failed))
      (progn
        (if (not arm-origin)
            (roslisp:call-service +service-name-add-point-cloud+ 'suturo_environment_msgs-srv:AddPointCloud :scenecam scenecam)
            (roslisp:call-service +service-name-add-point-cloud+ 'suturo_environment_msgs-srv:AddPointCloud :scenecam scenecam :arm_origin arm-origin)))))


  "
* Arguments
- obj-designator :: 
* Description

"
(def-action-handler focus-object (obj-designator)
  "Nothing ?")


  "
* Arguments
- desig :: 
* Description

"
(cpm:def-process-module suturo-planning-pm-perception (desig)
  (apply #'call-action (reference desig)))
