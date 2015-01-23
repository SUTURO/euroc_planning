(in-package :exec)

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

(defun task-selector (&optional (tsk "task1"))
  "Starts the plan for the task from the parameter server.

   If no task is set in the parameter server, the task given
   as argument will be started (default: task1)"
  (with-ros-node
    (let ((task (roslisp:get-param "/planning/task" tsk)))
      (funcall (symbol-function (read-from-string (format nil "exec:~a" task)))))))

(defun repeat (elem n)
  (when (> n 0)
    (cons (cram-designators:copy-designator elem) (repeat elem (- n 1)))))

(defun parse-target-zone (zone)
  (let* ((zone-descr (cdr zone))
         (expected-object (roslisp::get-xml-rpc-struct-member zone-descr ':|expected_object|))
         (max-distance (roslisp::get-xml-rpc-struct-member zone-descr ':|max_distance|))
         (target-position (roslisp::get-xml-rpc-struct-member zone-descr ':|target_position|)))
    (make-designator 'location `((expected-object ,expected-object)
                                 (max-distance ,max-distance)
                                 (pose ,(cl-tf:make-pose-stamped "/map"
                                                                 (ros-time)
                                                                 (cl-transforms:make-3d-vector (car target-position)
                                                                                               (cadr target-position)
                                                                                               (caddr target-position))
                                                                 (cl-transforms:make-identity-rotation)))))))

(def-top-level-cram-function task1 ()
  "Top level plan for task 1 of the euroc challenge"
  (let* ((task-variation (roslisp:get-param "/planning/task_variation" "1"))
         (xml-zones (roslisp::xml-rpc-struct-alist
                      (roslisp:get-param
                        (format nil "/task1_~a/public_description/target_zones"
                                    task-variation))))
         (target-zones (mapcar #'parse-target-zone xml-zones))
         ; TODO: get objects and locations from perceived world
         (obj-location (make-designator 'location `((pose ,(cl-tf:pose->pose-stamped "/map"
                                                             (ros-time)
                                                             (cl-transforms:make-identity-pose))))))
         (found-objects (repeat (make-designator 'object `((at ,obj-location)
                                                           (type cube))) (length target-zones)))
         (obj (car found-objects)))
    (with-designators
      ((obj-in-hand (object `((at ,(make-designator 'location '((gripper gripper)))))))
       (grasp-position (location '((to grasp)))))
      (let ((put-down-location (car target-zones)))
        (with-process-modules
          (mapcar (lambda (obj target-zone) ; TODO: probably needs better matching of objects to target zones
                    (roslisp:ros-info (task1) "Placing object ~a in target zone ~a" obj target-zone)
                    (at-location (grasp-position)
                      (achieve `(object-in-hand ,obj))
                      (equate obj obj-in-hand) ; object is now in gripper
                      (achieve `(object-put ,obj ,put-down-location))))
                  found-objects target-zones))))))


(def top-level-cram-function task1-tmp ()
  (plan)
  (print "Waiting")
  (cpl-impl:wait-for (fl-and (eql *current-state* :state-init) (eql *current-transition* :transition-successful)))
  (achieve `(map-scanned)))
