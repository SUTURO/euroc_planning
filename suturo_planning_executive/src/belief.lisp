(in-package :exec)

(defun yes (&rest args) t)

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
  
  (cram-reasoning:<- (looking-at ?pose)
    ; TODO: check if robot is looking at posititon
    (cram-reasoning:lisp-fun yes ?pose))

  (cram-reasoning:<- (object-picked ?object)
    (object-in-hand ?object))

  (cram-reasoning:<- (cram-plan-library:object-put ?object ?location)
    ; TODO: check if object was really put down at location
    (cram-reasoning:lisp-fun yes ?object ?location))

  (cram-reasoning:<- (holds ?occasion)
    (cram-reasoning:call ?occasion)))


