(in-package :planlib)

(defparameter *next_cluster* 0)

(defun scan-obstacle()
  (let ((region)
        (region-centroid)
        )
  (if (is-region-out-of-reach region)
      NIL))

(defun get-region-centroid(region)
  (let ((x)
        (y)
        (x-coord)
        (y-coord)
        (map)
        (cell-size)
        (map-size))
    (setf map (get-map))
    (setf map-size (roslisp:msg-slot-value (value map) 'size))
    (setf cell-size (roslisp:msg-slot-value (value map) 'cell_size))
    (setf x ())
    (
    

  (roslisp:make-msg "geometry_msgs/Point" (x) 2 (y) 1 (z) 1)
))

(defun get-avg (region)
  (let ((region-cell-coordinates)
        (number-of-cells)
        (avg-x 0)
        (avg-y 0))
    (setf region-cell-coordinates (cl-utilities:copy-array(roslisp:msg-slot-value region 'cells)))
    (setf number-of-cells (length region-cell-coordinates))
    
    (loop for cell across region-cell-coordinates do
      (+ avg-x (aref cell 0))
      (+ avg-y (aref cell 1)))
    
    (setf avg-x (/ avg-x number-of-cells))
    (setf avg-y (/ avg-y number-of-cells))

    (values avg-x avg-y)))

(defun get-map()
    (if (not (roslisp:wait-for-service +service-name-get-map+ *timeout-service*))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-get-map+))
      (roslisp:call-service +service-name-get-map+ 'std_msgs-msg:Empty)
      ))

(defun create-poses())

(defun calculate-distance(region)
  (+ (* (get-number-of-cells-from-current-region region) +scan-obstacles-distance-parameter-factor+) +scan-obstacles-distance-parameter-offset+))

(defun is-region-out-of-reach(region)
  (let ((distance-to-region))
    (setf distance-to-region (v-dist (make-3d-vector 0 0 0)(centroid-to-vector ))
    (> distance +scan-obstacles-arm-max-distance+) 
))


(defun get-number-of-cells-from-current-region(region) 
  (let ((cells (cl-utilities:copy-array(roslisp:msg-slot-value region 'cells))))
    (length cells)
))

(def-cram-function state-scan-obstacles ()
  (loop while T do
    (cpl-impl:wait-for (fl-and (eql *current-state* :state-search-objects) (eql *current-transition* :transition-search-objects)))
    (print "Executing state scan obstacles")
    (setf (value *current-state*) :state-scan-obstacles)
    (scan-obstacle)
    (setf (value *current-transition*) :transition-map-scanned)
    (setf (value *current-transition*) :transition-new-image)
        ))
