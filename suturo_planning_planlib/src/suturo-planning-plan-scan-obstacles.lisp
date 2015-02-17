(in-package :planlib)

(defun look-at-obstacle(region)
  (print "Scan-obstacle: Begin")
  (let ((region-centroid)
        (poses))
    (setf region-centroid (get-region-centroid region))
    (if (is-region-out-of-reach region-centroid)
        NIL)
    (setf poses (create-poses (calculate-distance region) region-centroid))
    (print "nmb poses")
    (print (length poses))
    (plan-and-move poses)))

(defun get-region-centroid(region)
    (multiple-value-bind (avg-x avg-y) (get-avg region)
      (multiple-value-bind (x y) (index-to-coordinates (+ avg-x -0.065) avg-y)
    
    (roslisp:make-msg "geometry_msgs/Point" (x) x (y) y))))

(defun get-avg (region)
  (let ((region-cell-coordinates)
        (number-of-cells)
        (avg-x 0)
        (avg-y 0))
    (setf region-cell-coordinates (get-region-cell-coordinates region))
    (print "get region-cell-coordinates:")
    (print region-cell-coordinates)
    (setf number-of-cells (length region-cell-coordinates))
    
    (loop for cell across region-cell-coordinates do
      (let ((cell-coords (roslisp:msg-slot-value cell 'data)))
      (setf avg-x (+ avg-x (aref cell-coords 0)))
      (setf avg-y (+ avg-y (aref cell-coords 1)))))
    
    (setf avg-x (/ avg-x number-of-cells))
    (setf avg-y (/ avg-y number-of-cells))

    (values avg-x avg-y)))

(defun get-region-cell-coordinates(region)
 (let ((region-cell-2d-array))
   (setf region-cell-2d-array (roslisp:msg-slot-value region 'cell_coords))
   (cl-utilities:copy-array(roslisp:msg-slot-value region-cell-2d-array 'data))
))

(defun index-to-coordinates(x-index y-index)
  (let ((x)
        (y)
        (map (get-map))
        (cell-size)
        (map-size))
    (setf cell-size (roslisp:msg-slot-value map 'cell_size))
    (setf map-size (roslisp:msg-slot-value (value map) 'size))
    (setf x (+(-(* x-index cell-size) (/ map-size 2)) (/ cell-size 2)))
    (setf y (+(-(* y-index cell-size) (/ map-size 2)) (/ cell-size 2)))
    (values x y)
))

(defun get-map()
    (if (not (roslisp:wait-for-service +service-name-get-map+ *timeout-service*))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-get-map+))
      (roslisp:msg-slot-value (roslisp:call-service +service-name-get-map+ 'suturo_environment_msgs-srv:GetMap) 'map)
      ))

(defun is-region-out-of-reach(region-centroid)
  (print region-centroid)
  (centroid-to-vector region-centroid)
  (let ((distance-to-region))
    (setf distance-to-region (v-dist (make-3d-vector 0 0 0)(centroid-to-vector region-centroid)))
    (> distance-to-region +scan-obstacles-arm-max-distance+) 
))

(defun calculate-distance(region)
  (+ (* (get-number-of-cells-from-current-region region) +scan-obstacles-distance-parameter-factor+) +scan-obstacles-distance-parameter-offset+))

(defun get-number-of-cells-from-current-region(region) 
  (let ((cells (cl-utilities:copy-array(roslisp:msg-slot-value region 'cells))))
    (length cells)
))

(defun create-poses(distance region-centroid)
  (roslisp:msg-slot-value (create-poses-service-call distance region-centroid) 'poses)
)

(defun create-poses-service-call(distance region-centroid)
  (if (not (roslisp:wait-for-service +service-name-create-poses-for-object-scanning+ *timeout-service*))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-create-poses-for-object-scanning+))
      (roslisp:call-service +service-name-create-poses-for-object-scanning+ 'suturo_manipulation_msgs-srv:CreatePosesForScanning :centroid region-centroid :angle +scan-obstacles-angle+ :distance distance :quantity +scan-obstacles-number-of-poses+)))

(defun plan-and-move(poses)
  (let ((not-blow-up-list (make-array 2 :fill-pointer 0)))
        (vector-push-extend "map" not-blow-up-list)
  (loop named poses-loop for pose across poses do
    (if (perform (make-designator 'action `((to move-arm-cam) ;TODO Es wird immer nur der erste ausgeführt, da immer ein response ausgegeben wird !!!!!!!!! 
                                            (pose ,pose)
                                            (do-not-blow-up-list ,not-blow-up-list))))
    (return-from poses-loop)))))

(def-cram-function state-scan-obstacles ()
  (loop while T do
    (cpl-impl:wait-for (fl-and (eql *current-state* :state-search-objects) (eql *current-transition* :transition-search-objects)))
    (print "Executing state scan obstacles")
    (setf (value *current-state*) :state-scan-obstacles)
    (scan-obstacles)
    (setf (value *current-transition*) :transition-map-scanned)
    (setf (value *current-transition*) :transition-new-image)
        ))
