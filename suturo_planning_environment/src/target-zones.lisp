(in-package :environment)

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

(defun get-target-zones ()
  (let* ((task-variation (roslisp:get-param "/planning/task_variation" "1"))
        (xml-zones (roslisp::xml-rpc-struct-alist
                     (roslisp:get-param
                       (format nil "/task1_~a/public_description/target_zones"
                                   task-variation)))))
    (mapcar #'parse-target-zone xml-zones)))

(defun find-matching-target-zone (object target-zones)
  (let ((obj-type (desig-prop-value object 'type)))
    (car (remove-if-not (lambda (zone)
                          (equal obj-type (desig-prop-value zone 'expected-object)))
                        target-zones))))
