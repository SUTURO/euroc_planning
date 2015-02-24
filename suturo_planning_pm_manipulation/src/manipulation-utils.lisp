(in-package :manipulation)

(defun transform-to (pose to-frame)
  (if (string= (roslisp:msg-slot-value (roslisp:msg-slot-value pose 'header) 'frame_id) to-frame) pose)
  (let ((odom-pose nil)
        (i 0)
        (transform-listener (make-instance 'cl-tf:transform-listener)))
    (loop 
      (if (typep pose 'moveit_msgs-msg:collisionobject)
        (progn
          (setf i 0)
          (let ((new-co (roslisp:modify-message-copy pose)))
            (dolist (cop (roslisp:msg-slot-value pose 'primitive-poses))
              (let ((p (roslisp:make-msg "geometry_msgs/PoseStamped"
                                         :header (roslisp:msg-slot-value pose 'header)
                                         (orientation pose) (roslisp:msg-slot-value cop 'orientation)
                                         (position pose) (roslisp:msg-slot-value cop 'position))))
                (setf p (transform-to p to-frame))
                (if (eql p nil) (return))
                (roslisp:with-fields (primitive-poses) new-co
                  (setf (nth i primitive-poses) (roslisp:modify-message-copy (nth i primitive-poses)
                                                                                  :position (roslisp:msg-slot-value (roslisp:msg-slot-value p 'pose) 'position)
                                                                                  :orientation (roslisp:msg-slot-value (roslisp:msg-slot-value p 'pose) 'orientation)))))
                (setf i (+ i 1)))
            (return (roslisp:modify-message-copy new-co :frame-id to-frame)))))
      (if (typep pose 'geometry_msgs-msg:posestamped)
          (setf odom-pose (cl-tf:transform-pose transform-listener :pose (cl-tf:msg->pose-stamped pose) :target-frame to-frame)))
      (if (typep pose 'geometry_msgs-msg:pointstamped)
          (setf odom-pose (cl-tf:transform-point transform-listener :point (cl-tf:msg->pose-stamped pose) :target-frame to-frame)))
      (if (not (eql odom-pose nil))
        (return odom-pose))                  
    (when (or (not (eql odom-pose nil)) (>= i 10)) (return)))
    (cl-tf:pose-stamped->msg odom-pose)))

(defun msg->quaternion (msg)
  (roslisp:with-fields (x y z w) msg
    (cl-transforms:make-quaternion x y z w)))

(defun quaternion->Point (q)
  (roslisp:make-msg "geometry_msgs/Point"
            :x (cl-transforms:x q)
            :y (cl-transforms:y q)
            :z (cl-transforms:z q)))

(defun quaternion->Quaternion (q)
  (roslisp:make-msg "geometry_msgs/Quaternion"
            :x (cl-transforms:x q)
            :y (cl-transforms:y q)
            :z (cl-transforms:z q)
            :w (cl-transforms:w q)))

(defun msg->vector (msg)
  (roslisp:with-fields (x y z) msg
    (cl-transforms:make-3d-vector x y z)))

(defun vector->msg (v)
  (with-slots (x y z) v
    (roslisp:make-msg "geometry_msgs/Point" :x x :y y :z z)))

(defun normalize-vector (v)
    (cl-transforms:v* v (/ 1.0 (cl-transforms:v-norm v))))

(defun magnitude (v)
  (cl-transforms:v-norm (msg->vector v)))

(defun rotate-quaternion (q roll pitch yaw)
  (let ((angle (cl-transforms:euler->quaternion :ax roll :ay pitch :az yaw)))
    (with-slots (x y z w) (cl-transforms:q* q angle)
      (roslisp:make-msg "geometry_msgs/Quaternion" :x x :y y :z z :w w))))

(defun add-point (v1 v2)
  (vector->msg (cl-transforms:v+ (msg->vector v1) (msg->vector v2))))

(defun qv_mult (q1 v1)
  (let ((q q1)
        (v v1))
    (if (typep v1 'geometry_msgs-msg:Point)
        (with-fields (x y z) v1
          (setf *v* (make-msg "geometry_msgs-msg/Quaternion" :x x :y y :z z :w 0))))
    (let ((r (quaternion_multiply (quaternion_multiply q v) (quaternion_conjugate q))))
      (make-msg "geometry_msgs-msg" :x (first r) :y (second r) :z (third r)))))

(defun quaternion_multiply (q v)
  (let ((q1 (msg->quaternion q))
        (v1 (msg->quaternion v)))
    (quaternion->Quaternion (cl-transforms:q* q1 v1))))

(defun three-points-to-quaternion (origin to &optional roll)
  (let ((roll-not-given nil))
    (if (eql roll nil)
        (setf roll (roslisp:make-msg "geometry_msgs/Point" :z (+ (roslisp:msg-slot-value origin 'z) 1.000001)))
        (setf roll-not-given T))
    (let ((n1 (normalize-vector (cl-transforms:v- (msg->vector to) (msg->vector origin))))
          (n (normalize-vector (cl-transforms:v- (msg->vector roll) (msg->vector origin)))))
      (let ((n2 (normalize-vector (cl-transforms:v- n (cl-transforms:v* n1 (cl-transforms:dot-product n n1))))))
        (let ((n3 (normalize-vector (cl-transforms:cross-product n1 n2))))
          (let ((q (quaternion->Quaternion (cl-transforms:matrix->quaternion (make-array '(4 4) :element-type 'float
                                                             :initial-contents `((,(x n1) ,(x n2) ,(x n3) 0.0)
                                                                                 (,(y n1) ,(y n2) ,(y n3) 0.0)
                                                                                 (,(z n1) ,(z n2) ,(z n3) 0.0)
                                                                                 (0.0 0.0 0.0 1)))))))
            (if roll-not-given
                (setf q (rotate-quaternion q (/ (- 0 pi) 2) 0 0)))
            q))))))

(defun get-angle (p1 p2)
  (let ((v1 (msg->vector p1))
        (v2 (msg->vector p2)))
    (acos (/ (dot-product v1 v2) (* (magnitude v1) (magnitude v2))))))

(defun get-pitch (q)
  (let ((direct (qv_mult q (make-msg "geometry_msgs/Point" :x 1 :y 0 :z 0))))
    (let ((v (modify-message-copy direct :z 0.0)))
      (if (and (<= 0 (magnitude v)) (<= (magnitude v) 0.001))
          (setf v (make-msg "geometry_msgs/Point" :x 1 :y 0 :z 0)))
    (get-angle v direct))))

(defun set-vector-length (l p)
  "Set the Length of a geometry_msgs-msg:Point p to the length l"
  (with-fields (x y z) p
    (v* (normalize-vector (make-3d-vector x y z)) l)))

(defun make-scan-poses (point distance angle frame n)
  (let ((look-positions (list))
        (alpha (- (/ pi 2) angle)))
    (let ((r (* (sin alpha) distance))
          (h (* (cos alpha) distance)))
      (let ((h-vector (cl-transforms:make-3d-vector 0 0 h)))
        (let ((muh (cl-transforms:v+ (msg->vector point) h-vector))
              (i 0))
          (loop
            (let ((a (* 2 (* pi (/ i n)))))
              (let ((b (- a (/ pi 2))))
                (let ((look-point (vector->msg (cl-transforms:v+ (set-vector-length r (cl-transforms:make-3d-vector (cos a) (sin a) 0)) muh)))
                      (roll-point (vector->msg (cl-transforms:v+ (cl-transforms:make-3d-vector (cos b) (sin b) 0) (msg->vector point)))))
                  (append look-positions (list (roslisp:make-msg "geometry_msgs/PoseStamped"
                                                                 (frame-id header) frame
                                                                 (orientation pose) (three-points-to-quaternion look-point point roll-point)
                                                                 (position pose) look-point)))))
              (when (>= i n) (return)))))))
    (sort look-positions (lambda (x) (magnitude (roslisp:msg-slot-value (roslisp:msg-slot-value x 'pose) 'position))))))

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


(defun get-place-positions (collision-object destination distance grasp)
	(with-fields (primitives) collision-object
		(if (eql (length primitives) 1)
			(get-single-object-place-positions collision-object destination distance grasp)
			(get-handlebar-place-positions collision-object destination distance))))


(defun get-single-object-place-positions (collision-object destination distance grasp)
  (with-fields (pose) grasp
    (with-fields (orientation position) pose
      (let ((angle (get-pitch orientation)))
        (let ((z (- (abs (msg-slot-value position 'z)) (* (sin angle) distance))))
          (let ((place-pose (modify-message-copy destination (+ z +safe-place+)))
                (diff (abs (- (/ pi 2) angle))))
            (let ((place-poses nil))
              (if (and (<= 0 diff) (<= diff 0.1))
                  (setf place-poses (make-scan-poses place-pose distance angle "/odom_combined" 2))
                  (setf place-poses (make-scan-poses place-pose distance angle "/odom_combined" 8)))
              (let ((p2 (transform-to (make-msg "geometry_msgs/PointStamped" (frame_id header) (msg-slot-value collision-object :id)) "tcp")))
                (let ((result (list)))
                  (if (< (msg-slot-value (msg-slot-value p2 'point) 'y) 0)
                      (dolist (p place-poses)
                        (with-fields (pose) p
                          (with-fields (orientation) pose
                            (append result (list (modify-message-copy p (orientation pose) (rotate-quaternion orientation pi 0 0))))))))
            result)))))))))


(defun get-handlebar-place-positions (collision-object destination distance)
  ; TODO implementieren
)


(defun get-pre-place-position (pose-stamped)
  (with-fields (pose) pose-stamped
	  (with-fields (position) pose
  	  (with-fields (z) position
    	  (modify-message-copy pose-stamped (z position pose) (+ z +pre-place-length+))))))
