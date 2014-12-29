(in-package :perception)

(defvar *current-state* "init" "The current state in the statemachine")
(defvar *name-node* "statemachine" "The name of the ros-node")
(defvar *name-service-init* "suturo/toplevel/init" "The name of the init service") 
(defvar *timeout-service* 10 "The time to wait for a service")

(defun plan ()
  (roslisp:with-ros-node (*name-node*)
    (call-init)))

(defun call-init ()
  (if (not (roslisp:wait-for-service *name-service-init* *timeout-service*))
           (roslisp:ros-warn nil "Timed out waiting for service init")
           (roslisp:call-service *name-service-init* 'suturo_interface_msgs-srv:StartPlanning)))
