(in-package :perception)

(defun search-objects ()
  (roslisp:with-ros-node ("two_ints_client")
    (if (not (roslisp:wait-for-service "search_objects" 10))
        (roslisp:ros-warn nil "Timed out waiting for service add_two_ints")  
        (roslisp:call-service "search_objects" 'suturo_interface_msgs-srv:SearchObjects))))
