(in-package :exec)

(defmethod cram-plan-knowledge:holds (occasion &optional time-specification)
  "Taken from simple_belief since the package pulls too many dependencies"
  (if time-specification
    (cram-reasoning:prolog `(holds ?_ ,occasion ,time-specification))
    (cram-reasoning:prolog `(holds ,occasion))))

(cram-reasoning:def-fact-group occasions (holds)
  (cram-reasoning:<- (object-in-hand ?object)
    (object-in-hand ?object ?_))
  
  (cram-reasoning:<- (object-in-hand ?object ?arms)
    (desig:desig-prop ?object (at ?obj-loc))
    (cram-reasoning:setof ?grip (desig:desig-prop ?obj-loc (gripper ?grip)) ?arms))
  
  (cram-reasoning:<- (holds ?occasion)
    (cram-reasoning:call ?occasion)))

(defun object-location-generator (location-designator)
  "Find the position of an object in an object-designator"
  (let* ((obj (desig-prop-value location-designator 'of))
         (obj-pos (desig-prop-value obj 'at)))
    (list obj-pos)))

(defun object-location-validator (location-designator pose)
  "Location from a designator is always a valid solution"
  :accept)

(defun central-location-generator (location-designator)
  (let ((pose (cl-tf:pose->pose-stamped "/map"
                                        (ros-time)
                                        (cl-transforms:make-identity-pose))))
  (list pose)))

(register-location-generator 15 object-location-generator)
(register-location-generator 15 central-location-generator)
(register-location-validation-function 15 object-location-validator)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
     (suturo-planning-pm-manipulation
      suturo-planning-pm-perception)
     ,@body))

(defmacro with-ros-node (&body body)
  "Utility macro to start and stop a ROS node around a block"
  `(prog2
     (roslisp-utilities:startup-ros)
     ,@body
     (roslisp-utilities:shutdown-ros)))

;;; TODO: Refactor the above into other packages

(defun main ()
  "Main function that executes when the executable is run"
  (task1))

(def-top-level-cram-function task1 ()
  "Top level plan for task 1 of the euroc challenge"
  (with-designators
    ((obj (object `((at ,(cl-tf:pose->pose-stamped "/map"
                                                    (ros-time)
                                                    (cl-transforms:make-identity-pose)))
                    (type cube))))
     (obj-in-hand (object `((at ,(make-designator 'location '((gripper gripper)))))))
     (put-down-location (location `((pose ,(cl-tf:pose->pose-stamped "/map"
                                                                     (ros-time)
                                                                     (cl-transforms:make-identity-pose))))))
     (grasp-position (location '((to grasp)))))
    (with-ros-node
      (with-process-modules
        (at-location (grasp-position)
          (achieve `(object-in-hand ,obj))
          (equate obj obj-in-hand) ; object is now in gripper
          (achieve `(object-put ,obj ,put-down-location)))))))
