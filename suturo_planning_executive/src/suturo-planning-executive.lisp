(in-package :exec)

(defmethod cram-plan-knowledge:holds (occasion &optional time-specification)
  ; TODO: Implement me (or try to use simple_belief from cram_gazebo with all [many] dependencies)
  "Implement me... somehow"
  nil)

(defun object-location-generator (location-designator)
  "Find the position of an object in an object-designator"
  (let* ((obj (desig-prop-value location-designator 'of))
         (obj-pos (desig-prop-value obj 'pose)))
    `(,obj-pos)))

(defun object-location-validator (location-designator pose)
  "Location from a designator is always a valid solution"
  :accept)

(register-location-generator 15 object-location-generator)
(register-location-validation-function 15 object-location-validator)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
     (suturo-planning-pm-manipulation
      suturo-planning-pm-perception)
     ,@body))

(defun main ()
  "Main function that executes when the executable is run"
  (task1))

(def-top-level-cram-function task1 ()
  "Top level plan for task 1 of the euroc challenge"
  (let ((obj (make-designator 'object
                              `((pose ,(cl-tf:pose->pose-stamped "/map"
                                                                 (ros-time)
                                                                 (cl-transforms:make-identity-pose)))
                                (type cube)))))
    (with-process-modules
      (seq
        (achieve `(object-picked ,obj))
        ; (achieve `(object-put ,obj ,loc)) ; Put-down plan does not run yet
        ))))
