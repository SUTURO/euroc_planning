(in-package :manipulation)

(defun transform-to (pose to-frame)
  "
  * Description
  Transform the given pose to the given frame

  * Arguments
    - pose :: The pose of to be converted :: geometry_msgs/PoseStamped
    - to-frame :: The name of the frame to which the pose should be converted :: string
  
  * Return
    - The transformed pose :: geometry_msgs/PoseStamped  
  "
  (if (string= (roslisp:msg-slot-value (roslisp:msg-slot-value pose 'header) 'frame_id) to-frame) pose)
  (let ((odom-pose nil)
        (i 0)
        (transform-listener cram-roslisp-common:*tf*))
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
          (setf odom-pose (cl-tf:pose-stamped->msg (cl-tf:transform-pose transform-listener :pose (cl-tf:msg->pose-stamped pose) :target-frame to-frame))))
      (if (typep pose 'geometry_msgs-msg:pointstamped)
          (setf odom-pose (cl-tf:point-stamped->msg (cl-tf:transform-point transform-listener :point (cl-tf:msg->point-stamped pose) :target-frame to-frame))))
      (if (not (eql odom-pose nil))
        (return odom-pose))                  
    (when (or (not (eql odom-pose nil)) (>= i 10)) (return)))
    odom-pose))

(defun msg->quaternion (msg)
  "
  * Description
  Convert the given message to a cl-transforms:Quaternion.

  * Arguments
    - msg :: The message which should be converted :: geometry_msgs/Quaternion

  * Return
    - The message as a new quaternion :: cl-transforms:Quaternion
  "
  (roslisp:with-fields (x y z w) msg
    (cl-transforms:make-quaternion x y z w)))

(defun quaternion->Point (q)
  "
  * Description
  Convert the given quaternion to a point.
  This function converts the given quaternion to a point by omitting the w-value.
  
  * Arguments
    - q :: The quaternion to convert to a point :: cl-transforms:Quaternion

  * Return
    - The quaternion converted to a point :: geometry_msgs/Point
  "
  (roslisp:make-msg "geometry_msgs/Point"
            :x (cl-transforms:x q)
            :y (cl-transforms:y q)
            :z (cl-transforms:z q)))

(defun quaternion->Quaternion (q)
  "
  * Description
  Convert the given cl-transforms:Quaternion to a quaternion message.
  
  * Arguments
    - q :: The quaternion to convert :: cl-transforms:Quaternion

  * Return
    - The converted quaternion as a message :: geometry_msgs/Quaternion
  "
  (roslisp:make-msg "geometry_msgs/Quaternion"
            :x (cl-transforms:x q)
            :y (cl-transforms:y q)
            :z (cl-transforms:z q)
            :w (cl-transforms:w q)))

(defun msg->vector (msg)
  "
  * Description
  Convert the given point to a 3D-Vector.
  
  * Arguments
    - msg :: The message which should be converted :: geometry_msgs/Point
  
  * Return
    - The converted message as a 3D-Vector :: cl-transforms:3d-vector
  "
  (roslisp:with-fields (x y z) msg
    (cl-transforms:make-3d-vector x y z)))

(defun vector->msg (v)
  "
  * Description
  Convert the given 3D-vector to a point message.

  * Arguments
    - v :: The 3D-Vector to be converted :: cl-transforms:3d-vector
  
  * Return
    - The converted 3D-Vector as a Point message :: geometry_msgs/Point
  "
  (with-slots (x y z) v
    (roslisp:make-msg "geometry_msgs/Point" :x x :y y :z z)))

(defun normalize-vector (v)
  "
  * Description
  Normalize the length of the given vector.

  * Arguments
    - v :: The vector to be normalized :: cl-transforms:3d-vector

  * Return
    - The normalized vector with a length of 1 :: cl-transforms:3d-vector
  "
  (cl-transforms:v* v (/ 1.0 (cl-transforms:v-norm v))))

(defun magnitude (v)
  "
  * Description
  Return the length of the given message.

  * Arguments
    - v :: The point message for which the length should be calculated :: geometry_msgs/Point

  * Return
    - The length of the given message :: float
  "
  (cl-transforms:v-norm (msg->vector v)))

(defun rotate-quaternion (q roll pitch yaw)
  "
  * Description
  Rotate the given quaternion message by the given values roll, pitch and yaw.

  * Arguments
    - q :: The quaternion which should be rotated :: geometry_msgs/Quaternion
    - roll :: The value of which the quaternion should be rolled :: float
    - pitch :: The value of which the quaternion should be pitched :: float
    - yaw :: The value of which the quaternion should be yawed :: float
  
  * Return
    - The rotated quaternion as a message :: geometry_msgs/Quaternion
  "
  (let ((clq (msg->quaternion q))
        (angle (cl-transforms:euler->quaternion :ax roll :ay pitch :az yaw)))
    (with-slots (x y z w) (cl-transforms:q* clq angle)
      (roslisp:make-msg "geometry_msgs/Quaternion" :x x :y y :z z :w w))))

(defun add-point (v1 v2)
  "
  * Description
  Add the given points.
  This operation is symmetric: add-point(v1 v2) = add-point(v2 v1).

  * Arguments
    - v1 :: The point which should be added to :: geometry_msgs/Point
    - v2 :: The point which should be added :: geometry_msgs/Point
  
  * Return
    - The addition of both points :: geometry_msgs/Point
  "
  (vector->msg (cl-transforms:v+ (msg->vector v1) (msg->vector v2))))

(defun quaternion-conjugate (q)
  "
  * Description
  Return the conjugation of the given quaternion.  

  * Arguments
    - q :: The quaternion for which the conjugation should be calculated :: geometry_msgs/Quaternion

  * Return
    - The conjugation of the given quaternion :: geometry_msgs/Quaternion
  "
  (with-fields (x y z w) q
    (make-msg "geometry_msgs/Quaternion" :x (* -1 x) :y (* -1 y) :z (* -1 z) :w w)))

(defun qv-mult (q1 v1)
  "
  * Description
  Rotate the given quaternion q1 by the given vector v1.

  * Arguments
    - q1 :: The quaternion to rotate :: geometry_msgs/Quaternion
    - v1 :: The vector to rotate the quaternion of :: geometry_msgs/Point || geometry_msgs/Quaternion
  
  * Return
    - The rotated quaternion :: geometry_msgs/Quaternion
  "
  (let ((q q1)
        (v v1))
    (if (typep v1 'geometry_msgs-msg:Point)
        (with-fields (x y z) v1
          (setf v (make-msg "geometry_msgs/Quaternion" :x x :y y :z z :w 0))))
    (let ((r (quaternion-multiply (quaternion-multiply q v) (quaternion-conjugate q))))
      (with-fields (x y z) r
        (make-msg "geometry_msgs/Point" :x x :y y :z z)))))

(defun quaternion-multiply (q v)
  "
  * Description
  Multiply the given quaternions via q*v
  
  * Arguments
    - q :: The quaternion to be multiplied :: geometry_msgs/Quaternion
    - v :: The quaternion to multiply with :: geometry_msgs/Quaternion
  
  * Return
    - The multiplication of the two quaternions :: geometry_mgs/Quaternion
  "
  (let ((q1 (msg->quaternion q))
        (v1 (msg->quaternion v)))
    (quaternion->Quaternion (cl-transforms:q* q1 v1))))


(defun three-points-to-quaternion (origin to &optional roll)
  "
  * Description
  Create a quaternion from the three given points

  * Arguments
    - origin :: The point which describes the origin of the quaternion :: geometry_msgs/Point
    - to :: The point which describes the coordinates to which the quaternion should point :: geometry_msgs/Point
    - roll :: (optional) The point which describes how the quaternion should be rolled :: geometry_msgs/Point

  * Return
    - The quaternion resulting from the three given points :: geometry_msgs/Quaternion
  "
  (let ((roll-not-given nil))
    (if (eql roll nil)
        (progn
          (setf roll (roslisp:make-msg "geometry_msgs/Point" :z (+ (roslisp:msg-slot-value origin 'z) 1.000001)))
          (setf roll-not-given T)))
    (let ((n1 (normalize-vector (cl-transforms:v- (msg->vector to) (msg->vector origin))))
          (n (normalize-vector (cl-transforms:v- (msg->vector roll) (msg->vector origin)))))
      (let ((n2 (normalize-vector (cl-transforms:v- n (cl-transforms:v* n1 (cl-transforms:dot-product n n1))))))
        (let ((n3 (normalize-vector (cl-transforms:cross-product n1 n2))))
          (let ((q (cl-transforms:matrix->quaternion (make-array '(4 4) :element-type 'float
                                                             :initial-contents `((,(x n1) ,(x n2) ,(x n3) 0.0)
                                                                                 (,(y n1) ,(y n2) ,(y n3) 0.0)
                                                                                 (,(z n1) ,(z n2) ,(z n3) 0.0)
                                                                                 (0.0 0.0 0.0 1))))))
            (if roll-not-given
              (rotate-quaternion q (/ (- 0 pi) 2) 0 0)
              (quaternion->Quaternion q))))))))

(defun get-angle (p1 p2)
  "
  * Description
  Return the angle between the two given points

  * Arguments
    - p1 :: The first point for which the angle should be calculated :: geometry_msgs/Point
    - p2 :: The second point for whic hthe angle should be calculated :: geometry_msgs/Point
  
  * Return
    - The angle between the two point in radiasn :: float
  "
  (let ((v1 (msg->vector p1))
        (v2 (msg->vector p2)))
    (acos (/ (dot-product v1 v2) (* (magnitude v1) (magnitude v2))))))

(defun get-pitch (q)
  "
  * Description
  Calculate the pitch of the given quaternion

  * Arguments
    - q :: The quaternion for which the pitch is calculated :: geometry_msgs/Quaternion

  * Return
    - The pitch of the given quaternion :: float
  "
  (let ((direct (qv-mult q (make-msg "geometry_msgs/Point" :x 1 :y 0 :z 0))))
    (let ((v (modify-message-copy direct :z 0.0)))
      (if (and (<= 0 (magnitude v)) (<= (magnitude v) 0.001))
          (setf v (make-msg "geometry_msgs/Point" :x 1 :y 0 :z 0)))
    (get-angle v direct))))


(defun set-vector-length (l p)
  "
  * Description
  Set the Length of the given p to the length l

  * Arguments
    - l :: The length to which the given point should be resized :: float
    - p :: The point which should be resized :: geometry_msgs/Point

  * Return
    - The rezied point :: cl-transforms:3d-vector
  "
  (with-fields (x y z) p
    (v* (normalize-vector (make-3d-vector x y z)) l)))

(defun mag (msg)
  "
  * Description
  Calculate the magnitude of the given pose

  * Arguments
    - msg :: The pose for which the magnitude should be calculated :: geometry_msgs/PoseStamped

  * Return
    - The magnitude of the given pose :: float
  "
  (magnitude (roslisp:msg-slot-value (roslisp:msg-slot-value msg 'pose) 'position)))

(defun make-scan-poses (point distance angle frame n)
  "
  * Description
  Calculate `n` scan poses for the given point with a given distance and a given angle.

  * Arguments
    - point :: The point which should be looked at :: geometry_msgs/Point
    - distance :: The distance from the scan-poses to the given point :: float
    - angle :: The angle of the scan-pose to the given point in radians :: float
    - frame :: The name of the tf frame the result should be transformed to :: string
    - n :: The number of scan poses to generate :: int
  
  * Return
    - A list of scan-poses for the given point :: (list geometry_msgs/PoseStamped)
  "
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
                  (setf look-positions (append look-positions (list (roslisp:make-msg "geometry_msgs/PoseStamped"
                                                                 (frame_id header) frame
                                                                 (orientation pose) (three-points-to-quaternion look-point point roll-point)
                                                                 (position pose) look-point)))))))
              (setf i (+ i 1))
              (when (>= i n) (return))))))
    (sort look-positions (lambda (x y) (< (mag x) (mag y))))))

(defun get-fingertip (hand-pose)
  "
  * Description
  Calculate the fingertip for a given hand pose

  * Arguments
    - hand-pose :: The pose of the hand :: geometry_msgs/PoseStamped
  
  * Return
    - The position of the fingertip :: geometry_msgs/PointStamped
  "
  (with-fields (pose) hand-pose
    (with-fields (position orientation) pose
      (with-fields (x y z) position
        (let ((pos (make-3d-vector x y z)))
          (with-fields (x y z w) orientation
            (let ((ori (normalize (make-quaternion x y z w))))
              (with-slots (x y z) (v+ (rotate ori (make-3d-vector (+ +hand-length+ +finger-length+) 0 0)) pos)
                (make-msg "geometry_msgs/PointStamped" (point) (make-msg "geometry_msgs/Point" :x x :y y :z z))))))))))

(defun get-pre-grasp (grasp)
  "
  * Description
  Calculate the pre-grasp pose for the given grasp pose.
  The pre-grasp pose is slightly above the given grasp pose.

  * Arguments
    - grasp :: The pose to grasp at :: geometry_msgs/PoseStamped
  
  * Return
    - The pre grasp pose slightly above the given pose :: geometry_msgs/PoseStamped
  "
  (with-fields (pose) grasp
    (with-fields (position) pose
      (with-fields (point) (get-fingertip grasp)
        (let ((p (make-3d-vector (msg-slot-value point 'x)
                                 (msg-slot-value point 'y)
                                 (msg-slot-value point 'z)))
              (pos (make-3d-vector (msg-slot-value position 'x)
                                   (msg-slot-value position 'y)
                                   (msg-slot-value position 'z))))
          (let ((new-p (vector->msg (v+ pos (set-vector-length +pre-grasp-length+ (v- pos p))))))
            (modify-message-copy grasp :pose (modify-message-copy pose :position new-p))
          )
        )
      )
    )
  )
)


(defun get-place-positions (collision-object destination distance grasp)
	"
  * Description
  Calculate a list of place poses for the given collision-object and destination.

  * Arguments
    - collision-object :: The object which should be placed :: moveit_msgs/CollisionObject
    - destination :: The destination where the object should be placed :: geometry_msgs/Point
    - distance :: The distance of the object to the destination point :: float
    - grasp :: The point where the object has been grasped :: geometry_msgs/PoseStamped
  
  * Return
    - A list of possible poses to place the object :: (list geometry_msgs/PoseStamped)
  "
  (with-fields (primitives) collision-object
		(if (eql (length primitives) 1)
      (progn
        (get-single-object-place-positions collision-object destination distance grasp))
      (progn
        (get-handlebar-place-positions collision-object destination distance)))))


(defun get-single-object-place-positions (collision-object destination distance grasp)
  "
  Calculate the place poses for a given collision-object with a single primitive, like a cube or a cylinder.
  For more informations see `get-place-positions`.

    * Arguments
    - collision-object :: The object which should be placed :: moveit_msgs/CollisionObject
    - destination :: The destination where the object should be placed :: geometry_msgs/Point
    - distance :: The distance of the object to the destination point :: float
    - grasp :: The point where the object has been grasped :: geometry_msgs/PoseStamped
  
  * Return
    - A list of possible poses to place the object :: (list geometry_msgs/PoseStamped)
  "
  (with-fields (pose) grasp
    (with-fields (orientation position) pose
      (let ((angle (get-pitch orientation)))
        (let ((z (- (abs (msg-slot-value position 'z)) (* (sin angle) distance))))
          (let ((place-pose (modify-message-copy destination :z (+ z +safe-place+)))
                (diff (abs (- (/ pi 2) angle))))
            (let ((place-poses nil))
              (if (and (<= 0 diff) (<= diff 0.1))
                  (setf place-poses (make-scan-poses place-pose distance angle "/odom_combined" 2))
                  (setf place-poses (make-scan-poses place-pose distance angle "/odom_combined" 8)))
              (let ((p2 (transform-to (make-msg "geometry_msgs/PointStamped" (frame_id header) (msg-slot-value collision-object :id)) "/tcp")))
                (let ((result (list)))
                  (if (< (msg-slot-value (msg-slot-value p2 'point) 'y) 0)
                      (progn
                        (dolist (p place-poses)
                          (with-fields (pose) p
                            (with-fields (orientation) pose
                              (setf result (append result (list (modify-message-copy p (orientation pose) (rotate-quaternion orientation pi 0 0))))))))
                        (setf place-poses result)))))
              place-poses)))))))


(defun get-handlebar-place-positions (collision-object destination distance)
  "
  * Description
  Calculate the place poses for a given collision-object which is a handlebar.
  For more informations see `get-place-positions`.

    * Arguments
    - collision-object :: The object which should be placed :: moveit_msgs/CollisionObject
    - destination :: The destination where the object should be placed :: geometry_msgs/Point
    - distance :: The distance of the object to the destination point :: float
  
  * Return
    - A list of possible poses to place the object :: (list geometry_msgs/PoseStamped)
  "
  (let ((angle 0)
        (p2 (transform-to (make-msg "geometry_msgs/PointStamped" (frame_id header) (msg-slot-value collision-object 'id)) "/tcp")))
    (let ((z1 (/ (aref (msg-slot-value (aref (msg-slot-value collision-object 'primitives) 0) 'dimensions) (symbol-code 'shape_msgs-msg:solidprimitive :CYLINDER_HEIGHT)) 2))
          (z2 (+ (aref (msg-slot-value (aref (msg-slot-value collision-object 'primitives) 1) 'dimensions) (symbol-code 'shape_msgs-msg:solidprimitive :BOX_X)) +safe-place+)))
      (let ((place-pose (modify-message-copy destination :z (+ (+ z1 (+ z2 +safe-place+) (abs (msg-slot-value (msg-slot-value p2 'point) 'y)))))))
        (let ((place-poses (make-scan-poses place-pose distance angle "/odom_combined" 8)))
          (if (< (msg-slot-value (msg-slot-value p2 'point) 'y) 0)
              (progn
                (let ((result (list)))
                  (loop for place-pose in place-poses do
                    (with-fields (pose) place-pose
                      (with-fields (orientation) pose
                        (setf result (append result (list (modify-message-copy place-pose :pose (modify-message-copy pose :orientation (rotate-quaternion orientation pi 0 0)))))))))
                  (setf place-poses result))))
          place-poses)))))


(defun get-pre-place-position (pose-stamped)
  "
  * Description
  Calculate the pre-place pose for the given place pose.

  * Arguments
    - pose-stamped :: The pose where the object should be places :: geometry_msgs/PoseStamped

  * Return
    - The pre place pose for the given pose :: geometry_msgs/PoseStamped
  "
  (with-fields (pose) pose-stamped
	  (with-fields (position) pose
  	  (with-fields (z) position
    	  (modify-message-copy pose-stamped (z position pose) (+ z +pre-place-length+))))))
