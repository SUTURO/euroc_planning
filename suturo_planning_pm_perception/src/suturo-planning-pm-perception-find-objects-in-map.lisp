(in-package :perception)


(def-action-handler find-objects-in-map (objects)
  (let ((regions (get-regions))
        (classified-regions (make-array 0 :fill-pointer 0 :adjustable t)))
    (compare-object-and-regions objects regions classified-regions)
    classified-regions)) ;TODO temporary solution: DO NOT USE GLOBAL VARIABLE

(defun get-regions()
  (if (not (roslisp:wait-for-service +service-name-get-obstacle-regions+ +timeout-service+))
      (roslisp:ros-warn nil t (concatenate 'string "Following service timed out: " +service-name-get-obstacle-regions+))
      (roslisp:msg-slot-value (roslisp:call-service +service-name-get-obstacle-regions+ 'suturo_environment_msgs-srv:GetObstacleRegions) 'obstacle_regions)))

(defun compare-object-and-regions(yaml-objects regions classified-regions)
  (let ((regions-with-same-color)
        (count-regions-with-same-color)
        )
    (loop for obj across yaml-objects do
      (setf regions-with-same-color (find-regions-with-same-color obj regions))
      (setf count-regions-with-same-color (length regions-with-same-color))
      (if (= count-regions-with-same-color 0)
          (print "No Region found for Object"))
      (if (= count-regions-with-same-color 1)
          (add-region-to-classified-regions (aref regions-with-same-color 0) classified-regions))
      (if (> count-regions-with-same-color 1)
          (print "Too many regions found for Object. No error handling for this yet")))))

(defun find-regions-with-same-color(obj regions)
  (let ((regions-with-same-color (make-array 0 :fill-pointer 0 :adjustable t))
        (obj-color (get-obj-color obj))
        (region-color))
    (loop for region across regions do
      (setf region-color (get-region-color region))
      (if (string= region-color obj-color) 
          (progn
            (print "TRUE")
            (vector-push-extend region regions-with-same-color))))
    regions-with-same-color))

;;(defun handle-multiple-regions-in-array(regions-with-same-color object)
;;(let ((dimensions (make-array 0 :fill-pointer 0 :adjustable t))
;;      (h)
;;      (region-with-same-high))
;;  (loop for primitive across (roslisp:msg-slot-value object 'primitives) do
;;    (if (or (= (length (roslisp:msg-slot-value object 'primitives)) 1)
;;            (= (roslisp:msg-slot-value primitive 'type) 1) ;;1 = SolidPrimitive.BOX
;;            (vector-push-extend (roslisp:msg-slot-value primitive 'dimensions))))  
;;        (print "test"))))


(defun add-region-to-classified-regions(region classified-regions)
  (vector-push-extend region classified-regions)
)

(defun get-obj-color(obj)
  (roslisp:msg-slot-value (value obj) 'color)
)

(defun get-region-color(region)
  (roslisp:msg-slot-value (value region) 'color_hex)
)
