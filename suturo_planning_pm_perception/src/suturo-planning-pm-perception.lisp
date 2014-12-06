(in-package :pm-perception)

(defun create-capability-string(&optional (cuboid 1) (pose-estimation nil) (object-ids nil))
  (let ((perception-capabilities "height,centroid,color"))
    (when cuboid
      (setf perception-capabilities (concatenate 'string perception-capabilities ",cuboid")))
    (when pose-estimation
      (setf perception-capabilities (concatenate 'string perception-capabilities ",ModelPoseEstimation"))
      (when object-ids
        (let ((s-ids "")) 
          (loop for id in object-ids do
                (setf s-ids (concatenate 'string s-ids (write-to-string id) "," )))
          (setf s-ids (concatenate 'string "(" (subseq s-ids 0 (-(length s-ids) 1)) ")" ))
          (setf perception-capabilities (concatenate 'string perception-capabilities s-ids)))))
    (return-from create-capability-string perception-capabilities)))
