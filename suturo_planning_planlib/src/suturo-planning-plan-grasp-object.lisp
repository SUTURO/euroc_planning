(in-package :planlib)


(defun grasp-object()
  (roslisp:with-ros-node (*name-node*)
    (setf (value *current-state*) :state-grasp-object)
    (if (not (roslisp:wait-for-service +service-name-grasp-object *timeout-service*))
        (roslisp:ros-warn nil t (concatenate 'string "Following service times out: ", +service-name-grasp-object+))
        (let ((msg (roslisp:call-service +service-name-grasp-object+ 'suturo_interface_msgs-srv:TaskDataService :taskdata (value *taskdata*))
		(setf (value *taskdata*) (roslisp:msg-slot-value msg 'taskdata))
		(setf (value *current-transition*) (string-transition-to-keyword (roslisp:msg-slot-value msg 'result))
   )
  )
)
 
