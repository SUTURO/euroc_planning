(in-package :planlib)

(defparameter *classified_regions* (make-array 0 :fill-pointer 0 :adjustable t) "Vector that contains the classified regions")
(defparameter *next_cluster* 0)

(defparameter *obstacle_cluster*)
(defparameter *base*)
(defparameter *current_region*)
(defparameter *region_centroid*)
(defparameter *dist_to_region*)
(defparameter *poses*)
(defparameter *current_pose*)
(defparameter *region_to_eef*)
(defparameter *plan*)

(defconstant *angle*)
(defconstant *distance*)


(def-cram-function state-scan-obstacles ()
  (loop while T do
        (cpl-impl:wait-for (fl-and (eql *current-state* :state-search-objects) (eql *current-transition* :transition-search-objects)))
        (print "Executing state scan obstacles")
        (setf (value *current-state*) :state-scan-obstacles)
        (scan-obstacles)
        (setf (value *current-transition*) :transition-map-scanned)
        (setf (value *current-transition*) :transition-new-image)
        )
)

(defun scan-obstacles(
(if ((roslisp:msg-slot-value (value *taskdata*) 'sec_try_done))
    (
      (setf (*current_region*) (aref *classified_region* (*next_cluster*-1) 0))
      (setf-msg (value *taskdata*) (sec_try) T)
      )
    (
    ;Get Regions
     (if (=(length *classified_regions*) 0)
         (setf (*obstacle_cluster*) (roslisp:msg-slot-value (utils:map:get-obstacle-regions)) 'obstacle_regions)
         (setf (*classified_regions*) (roslips:msg-slot-value (utils:map:undercover-classifier(obstacle_cluster, userdata.yaml.objects))) 'undercover_classifier) ;Pruefen!
         (setf (*base*) (roslisp:msg-slot-value (utils:manipulation:get-base-origin)) 'base_origin)
;Woher bekomme ich dies? self.classified_regions.sort(key=lambda x: euclidean_distance(Point(*(utils.map.index_to_coordinates(*x[0].get_avg()))+(0.0,)), base))
      )

  (if (>= (length *classified_regions*) *next_cluster*)
      (print "Searched all clusters")
      (setf (transition :transition-no-regions-left))
      (setf (*current_region*) (aref classified_regions *next_cluster* 0))
      ((*next_cluster*) += 1)
      (*next_cluster* +1)

  ;Ueb: region_centroid = Point(*(utils.map.index_to_coordinates(*current_region.get_avg()))+(-0.065,))
      (setf (*region_centroid*) ())
  ;Ueb: dist_to_region = mathemagie.euclidean_distance(Point(0, 0, 0), region_centroid)
      (setf (*dist_to_region*) ())
      )
  (if (not (roslisp:msg-slot-value (value *taskdata*)) 'enable_movement)
      (if (> (*dist_to_region*) 1.1) (setf (transition :transition-map-scanned)))

      (setf (*angle*) 1.2)
      ;distance = 0.6 + current_region.get_number_of_cells()*0.008
      (setf (*distance*) 0.6)

;poses = make_scan_pose(region_centroid, distance, angle, n=16)
(setf (*poses*) (roslisp: call-service +service-name-make-scan-pose+ )) ;!
(if (not (roslisp:msg-slot-value (value *taskdata*)) 'enable_movement)
    ; poses = utils.manipulation.filter_close_poses(poses)
    (setf (*poses*) (roslisp: call-service +service-name-filter-close-poses+ ()));!

    ;poses = utils.map.filter_invalid_scan_poses2(region_centroid.x, region_centroid.y, poses)
    (setf (*poses*) (roslisp: call-service +service-name-filter-invalid-scan-poses2+ ));!
)

(if ((roslisp:msg-slot-value (value *taskdata*)) 'sec_try_done)
   ; current_pose = utils.manipulation.get_eef_position().pose.position
    (roslisp: call-service +service-name-get-eef-position+ )
   ; current_pose.z = 0

   ; region_to_eef = subtract_point(region_centroid, current_pose)

   ; poses.sort(key=lambda pose: abs(get_angle(region_to_eef, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), region_centroid)) - pi/2))

   ; visualize_poses(poses)
)

(if (roslisp:msg-slot-value (value *taskdata*) 'enable_movement)
    (
     ; plan = utils.manipulation.plan_arm_and_base_to
     (setf (plan) (roslisp: msg-slot-value (value utils:manipulation)) 'plan_arm_and_base_to)
)
    (
     ; plan = utils.manipulation.plan_arm_to
     (setf (plan) (roslisp: msg-slot-value (value utils:manipulation)) 'plan_arm_to)
     ;utils.manipulation.blow_up_objects(do_not_blow_up_list=("map"))
     ()
       ; for pose in poses:        
         ;   if utils.manipulation.move_with_plan_to(plan(pose)):
      ; userdata.focused_point = region_centroid

               ; rospy.logdebug('Wait for clock')
               ; time.sleep(0.5)
               ; rospy.sleep(2.5)
               ; return 'newImage'
(loop for pose in *poses* do
     if( ( )
         (setf-msg (value *taskdata*) (focused_point) *region_centroid* )
         (print "Wait for clock")
         (sleep 0.5)
         ;rospy sleep?
         (sleep 2.5)
         (setf (transition :transition-new-image))
         )
       )
       )
)

(roslisp: call-service +service-name-blow-down-objects+ );!
(setf (transition :transition-map-scanned))
)
 
)
 )
)
)