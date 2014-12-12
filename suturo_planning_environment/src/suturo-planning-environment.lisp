(in-package :environment)

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
    (list (make-designator 'location `((pose ,pose))))))

(register-location-generator 15 object-location-generator)
(register-location-generator 15 central-location-generator)
(register-location-validation-function 15 object-location-validator)

(defmethod designators-ros::ensure-pose-stamped ((location-designator location-designator) frame-id stamp)
  (declare (ignore frame-id stamp))
  (desig-prop-value location-designator 'pose))
