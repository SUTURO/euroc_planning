(in-package :planlib)

(defparameter *classified_regions* (make-array 0 :fill-pointer 0 :adjustable t) "Vector that contains the classified regions")
(defparameter *next_cluster* 0)

(defun scan-obstacles()
  (if (is-region-out-of-reach) 
)

(defun is-region-out-of-reach()
  (let ((max_distance 1.1)
        (distance))
    (setf distance (+ (* (get-number-of-cells-from-current-region) 0.008) 0.6))
    (> distance max_distance) 
))

(defun get-number-of-cells-from-current-region() 
  1
)

(def-cram-function state-scan-obstacles ()
  (loop while T do
    (cpl-impl:wait-for (fl-and (eql *current-state* :state-search-objects) (eql *current-transition* :transition-search-objects)))
    (print "Executing state scan obstacles")
    (setf (value *current-state*) :state-scan-obstacles)
    (scan-obstacles)
    (setf (value *current-transition*) :transition-map-scanned)
    (setf (value *current-transition*) :transition-new-image)
        ))
