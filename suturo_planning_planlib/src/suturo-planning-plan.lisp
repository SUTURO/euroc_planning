(in-package :planlib)

(defvar *current-state* "init" "The current state in the statemachine")
(defvar *current-transition* "start" "The transition returned from a state")
(defvar *taskdata* nil "The needed data for the state machine")
(defvar *name-node* "statemachine" "The name of the ros-node")
(defvar *name-service-init* "suturo/toplevel/init" "The name of the init service") 
(defvar *timeout-service* 10 "The time to wait for a service")
(defvar *state* (make-fluent :name :state) "The current state")

(defun plan ()
  (roslisp:with-ros-node (*name-node*)
    (loop while (not (string= *current-state* "end")) do
      (cond
        ((and (string= *current-state* "init") (string= *current-transition* "start"))  (call-init))
        ;;((and (string= *current-state* "init") (string= *current-transition* "successful")) (call-service-state "yaml_handler")) 
        ((and (string= *current-state* "init") (string= *current-transition* "successful")) (call-service-state "scan_map")) 
        ((and (string= *current-state* "scan_map") (string= *current-transition* "mapScanned" )) (call-service-state "scan_shadow")) 
        ((and (string= *current-state* "scan_shadow") (string= *current-transition* "success" )) (call-service-state "search_objects")) 
        ((and (string= *current-state* "search_objects") (string= *current-transition* "missingObjects" )) (call-service-state "scan_obstacles"))
        ((and (string= *current-state* "search_objects") (string= *current-transition* "noObjectsLeft" )) (done))
        ((and (string= *current-state* "scan_obstacles") (string= *current-transition* "mapScanned" )) (call-service-state "scan_obstacles")) 
        ((and (string= *current-state* "scan_obstacles") (string= *current-transition* "newImage" )) (call-service-state "classify_objects")) 
        ((and (string= *current-state* "scan_obstacles") (string= *current-transition* "noRegionLeft" )) (done)) 
        ((and (string= *current-state* "classify_objects") (string= *current-transition* "objectsClassified" )) (call-service-state "focus_objects")) 
        ((and (string= *current-state* "classify_objects") (string= *current-transition* "noObject" )) (call-service-state "search_objects")) 
        ((and (string= *current-state* "focus_objects") (string= *current-transition* "success" )) (call-service-state "search_objects")) 
        ((and (string= *current-state* "focus_objects") (string= *current-transition* "fail" )) (call-service-state "search_objects")) 
        ((and (string= *current-state* "focus_objects") (string= *current-transition* "focusHandle" )) (call-service-state "pose_estimate_object")) 
        ((and (string= *current-state* "focus_objects") (string= *current-transition* "focusObject" )) (call-service-state "pose_estimate_object")) 
        ((and (string= *current-state* "pose_estimate_object") (string= *current-transition* "success" )) (call-service-state "focus_objects")) 
        ((and (string= *current-state* "pose_estimate_object") (string= *current-transition* "fail" )) (call-service-state "focus_objects")) ))))

(defun call-init ()
  (format t "Calling init \n")
  (if (not (roslisp:wait-for-service *name-service-init* *timeout-service*))
           (progn 
             (roslisp:ros-warn nil "Timed out waiting for service init")
             (setf *current-transition* "timed-out"))
           (progn 
             (setf *taskdata* (roslisp:call-service *name-service-init* 'suturo_interface_msgs-srv:StartPlanning))
             (setf *current-transition* "successful"))))

(defun call-service-state (service-name)
  (let
      ((full-service-name (concatenate 'string "suturo/state/" service-name)))
    (format t (concatenate 'string "calling service: " service-name))
    (if (not (roslisp:wait-for-service full-service-name *timeout-service*))
        (let 
            ((timed-out-text (concatenate 'string "Timed out waiting for service " service-name)))
          (roslisp:ros-warn nil t timed-out-text)
          (setf *current-transition* "timed-out"))
        (progn
          (setf *current-state* service-name)
          (let 
              ((value (roslisp:call-service full-service-name 'suturo_interface_msgs-srv:TaskDataService :taskdata *taskdata*))) 
            (setf *taskdata* (roslisp:msg-slot-value value 'taskdata))
            (setf *current-transition* (roslisp:msg-slot-value value 'result))
            )))))

(defun done ()
  (format t "Done")
  (setf *current-state* "end"))
