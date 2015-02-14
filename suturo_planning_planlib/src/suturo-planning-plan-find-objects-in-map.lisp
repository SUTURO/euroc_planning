(in-package :planlib)

(defparameter *regions* (make-array 0 :fill-pointer 0 :adjustable t) "Vector that contains the regions. They are not classified.")
(defparameter *classified-regions* (make-array 0 :fill-pointer 0 :adjustable t) "Vector that contains the classified regions")

;; *found-objects* sollte muss schon gefüllt sein

(defun find-objects-in-map()
  (let ((regions (get-regions))
        (yaml-objects (cl-utilities:copy-array(roslisp:msg-slot-value (roslisp:msg-slot-value (value *taskdata*) 'yaml) 'objects))))
    (compare-object-and-regions yaml-objects regions)
    ))

(defun get-regions()
  (if (not (roslisp:wait-for-service +service-name-get-obstacle-regions+ +timeout-service+))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-get-obstacle-regions+))
      (roslisp:msg-slot-value (roslisp:call-service +service-name-get-obstacle-regions+ 'suturo_environment_msgs-srv:GetObstacleRegions) 'obstacle_regions)
      ))

(defun compare-object-and-regions(yaml-objects regions)
  (let ((regions-with-same-color)
        (count-regions-with-same-color)
        )
    (loop for obj across yaml-objects do
      (setf regions-with-same-color (find-regions-with-same-color obj regions))
      (setf count-regions-with-same-color (length regions-with-same-color))
      (if (= count-regions-with-same-color 0)
          (print "No Region found for Object"))
      (if (= count-regions-with-same-color 1)
          (add-region-to-classified-regions (aref regions-with-same-color 0)))
      (if (> count-regions-with-same-color 1)
          (print "Too many regions found for Object. No error handling for this yet"))
)))

(defun find-regions-with-same-color(obj regions)
  (let ((regions-with-same-color (make-array 0 :fill-pointer 0 :adjustable t))
        (obj-color (get-obj-color obj))
        (region-color))
    (loop for region across regions do
      (setf region-color (get-region-color region))
      (if (string= region-color obj-color) 
          (vector-push-extend region regions-with-same-color)))
    regions-with-same-color
))

(defun add-region-to-classified-regions(region)
  (vector-push-extend region *classified-regions*)
)

(defun get-obj-color(obj)
  (roslisp:msg-slot-value (value obj) 'color)
)

(defun get-region-color(region)
  (roslisp:msg-slot-value (value region) 'color_hex)
)

(def-cram-function state-find-objects-in-map ()
  (loop while T do
    (cpl-impl:wait-for (fl-and (eql *current-state* :state-search-objects) (eql *current-transition* :transition-search-objects)))
    (print "Executing state scan obstacles")
    (setf (value *current-state*) :state-scan-obstacles)
    (find-objects-in-map)
    (setf (value *current-transition*) :transition-map-scanned)
    (setf (value *current-transition*) :transition-new-image)
        ))
