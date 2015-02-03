(in-package :exec)

(defun yes (&rest args) t)

(defun no (&rest args) nil)

(defun is-robot-looking-at (location)
  "Checks if the robot is looking at the given location"
  ; TODO: Needs implementation
  nil
  )

(defun is-robot-at-location (temp-loc)
  (let* ((robot-pose (environment:get-model-pose "pr2"))
         (temp-pose (desig:reference temp-loc)))
    (poses-equal-p temp-pose robot-pose)))

(defun poses-equal-p (pose-1 pose-2 &key
                             (position-threshold 0.01)
                             (angle-threshold (/ pi 180)))
  (declare (type cl-transforms:pose pose-1 pose-2))
  (and (< (tf:v-dist (tf:origin pose-1) (tf:origin pose-2)) position-threshold)
       (< (tf:angle-between-quaternions
            (tf:orientation pose-1)
            (tf:orientation pose-2))
            angle-threshold)))

(defun is-map-scanned()
  "TODO Set timeout "
  (let ((is-scanned nil)
        (percentage 0))
    (print "Im in is map scanned")
    (if (not (roslisp:wait-for-service "suturo/map/get_percent_cleared" 1))
        (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " "suturo/map/get_percent_cleared" ))
        (progn
          (print "calling service percent cleared")
          (setf percentage (roslisp:msg-slot-value (roslisp:call-service  "suturo/map/get_percent_cleared" 'suturo_interface_msgs-srv:GetPercentCleared) 'percent))
          (print "Service call done")
          (format t "~$" percentage)
          (if (> percentage 0.95)
              (setf is-scanned T))))
    (print "Returning is scanned")
    is-scanned))


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
  
  (cram-reasoning:<- (looking-at ?location)
    (cram-reasoning:lisp-pred is-robot-looking-at ?location))

  (cram-reasoning:<- (object-picked ?object)
    (object-in-hand ?object))

  (cram-reasoning:<- (cram-plan-library:object-put ?object ?location)
    ; TODO: check if object was really put down at location
    (cram-reasoning:lisp-fun yes ?object ?location))

  (cram-reasoning:<- (map-scanned)
    (cram-reasoning:lisp-pred is-map-scanned)
    (print "Im doooonnnneee !!!!!!!!!!")
    ;;TODO: Check if 95% of the map is scanned
   )

  (cram-reasoning:<- (objects-informed)
    ; TODO: Implement me
    (cram-reasoning:lisp-pred no))

  (cram-reasoning:<- (objects-in-place ?objects)
    ; TODO: Implement me
    (cram-reasoning:lisp-pred no))

  (cram-reasoning:<- (holds ?occasion)
    (cram-reasoning:call ?occasion)))


