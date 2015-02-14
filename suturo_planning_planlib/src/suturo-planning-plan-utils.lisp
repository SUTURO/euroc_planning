(in-package :planlib)

(defun get-euclidean-distance-from-centroids(centroid1 centroid2)
  (let (
        (v1 (centroid-to-vector centroid1)) 
        (v2 (centroid-to-vector centroid2)))
    (v-dist v1 v2)
))

(defun centroid-to-vector(centroid)
  (let ((3d-vector (make-3d-vector (roslisp:msg-slot-value centroid 'x) (roslisp:msg-slot-value centroid 'y) (roslisp:msg-slot-value centroid 'z))))
    
))
