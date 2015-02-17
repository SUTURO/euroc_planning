(in-package :manipulation)


(defun normalize-vector (v)
    (v* v (/ 1.0 (v-norm v)))
)


(defun set-vector-length (l p)
  "Set the Length of a geometry_msgs-msg:Point p to the length l"
  (with-fields (x y z) p
    (v* (normalize-vector (make-3d-vector x y z)) l)
  )
)

(defun get-fingertip (hand-pose)
  (with-fields (pose) hand-pose
    (with-fields (position orientation) pose
      (with-fields (x y z) position
        (let ((pos (make-3d-vector x y z)))
          (with-fields (x y z w) orientation
            (let ((ori (normalize (make-quaternion x y z w))))
              (with-slots (x y z) (v+ (rotate ori (make-3d-vector (+ +hand-length+ +finger-length+) 0 0)) pos)
                (make-msg "geometry_msgs/PointStamped" (point) (make-msg "geometry_msgs/Point" :x x :y y :z z)))))))))
)

(defun get-pre-grasp (grasp)
  (with-fields (pose) grasp
    (with-fields (position) pose
      (with-fields (point) (get-fingertip grasp)
        (let ((p (make-3d-vector (msg-slot-value point 'x)
                                 (msg-slot-value point 'y)
                                 (msg-slot-value point 'z)))
              (pos (make-3d-vector (msg-slot-value position 'x)
                                   (msg-slot-value position 'y)
                                   (msg-slot-value position 'z))))
          (let ((new-p (v+ pos (set-vector-length +pre-grasp-length+ (v- pos p)))))
            (modify-message-copy grasp :pose (modify-message-copy pose :position new-p))
          )
        )
      )
    )
  )
)

(defun get-pre-place-position (pose)
  (with-fields (position) pose
    (with-fields (z) position
      (modify-message-copy pose (z position) (+ z +pre-place-length+)))))
