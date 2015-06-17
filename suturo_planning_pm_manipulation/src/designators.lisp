(in-package :manipulation)

(cram-reasoning:def-fact-group manipulation-action-designator (action-desig)
  "This function defines the conditions which are used in the prolog-reasoning-engine to find the proper action-handler for the given action-desig.
Condition are build as following:
(cram-reasoning:<- (action-desig ?designator (action-handler-that-should-be-executed action-handler-param1 action-handler-param2 ...)
                                 (desig-prop ?designator (key1 value1)) ;assumes the designator contains a (key1 value1) pair
                                 ... ))
*Arguments
- action-desig :: The action-designators which should be resolved"
  (cram-reasoning:<- (action-desig ?designator (follow ?pose))
                     (desig-prop ?designator (to follow))
                     (desig-prop ?designator (pose ?pose)))

  (cram-reasoning:<- (action-desig ?designator (grasp ?obj))
                     (desig-prop ?designator (to grasp))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (move-mast-cam ?pan ?tilt))
                     (desig-prop ?designator (to move-mast-cam))
                     (desig-prop ?designator (pan ?pan))
                     (desig-prop ?designator (tilt ?tilt)))

  (cram-reasoning:<- (action-desig ?designator (navigation ?pose))
                     (desig-prop ?designator (to move-arm-cam))
                     (desig-prop ?designator (pose ?pose)))

  (cram-reasoning:<- (action-desig ?designator (navigation ?pose ?do-not-blow-up-list))
                     (desig-prop ?designator (to move-arm-cam))
                     (desig-prop ?designator (pose ?pose))
                     (desig-prop ?designator (do-not-blow-up-list ?do-not-blow-up-list)))
 
  (cram-reasoning:<- (action-desig ?designator (move-arm-cam-pose-name ?pose-name))
                     (desig-prop ?designator (to move-arm-cam))
                     (desig-prop ?designator (pose-name ?pose-name)))

  (cram-reasoning:<- (action-desig ?designator (lift nil nil))
                     (desig-prop ?designator (to lift))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (carry ?obj))
                     (desig-prop ?designator (to carry))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (open-gripper ?position))
                     (desig-prop ?designator (to open-gripper))
                     (desig-prop ?designator (position ?position)))

  (cram-reasoning:<- (action-desig ?designator (no-navigation ?goal))
                     (desig-prop ?designator (type navigation))
                     (desig-prop ?designator (goal ?goal)))

  (cram-reasoning:<- (action-desig ?designator (put-down ?cobj ?pose ?grasp))
                     (desig-prop ?designator (to put-down))
                     (desig-prop ?designator (obj ?obj))
                     (desig-prop ?obj (collision-object ?cobj))
                     (desig-prop ?designator (at ?loc))
                     (desig-prop ?loc (pose ?pose))
                     (desig-prop ?obj (grasp-position ?grasp)))

  (cram-reasoning:<- (action-desig ?designator (park ?obj))
                     (desig-prop ?designator (to park))
                     (desig-prop ?designator (obj ?obj))))

(cram-reasoning:def-fact-group manipulation-actions (cram-process-modules:matching-process-module
                                                     cram-process-modules:available-process-module)
  "Defines which action (key:to,value:action) belongs to the process module suturo-planning-pm-manipulation"
  (cram-reasoning:<- (cram-process-modules:matching-process-module ?designator suturo-planning-pm-manipulation)
                     (or (desig-prop ?designator (to follow))
                         (desig-prop ?designator (to grasp))
                         (desig-prop ?designator (to open-gripper))
                         (desig-prop ?designator (to lift))
                         (desig-prop ?designator (to carry))
                         (desig-prop ?designator (type navigation))
                         (desig-prop ?designator (to put-down))
                         (desig-prop ?designator (to park))
                         (desig-prop ?designator (to move-arm-cam)) 
                         (desig-prop ?designator (to move-mast-cam))))
  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-manipulation)))
