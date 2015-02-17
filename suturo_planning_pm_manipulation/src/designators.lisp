(in-package :manipulation)

(cram-reasoning:def-fact-group manipulation-action-designator (action-desig)
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

  (cram-reasoning:<- (action-desig ?designator (lift ?grasp-point ?collision-object-name))
                     (desig-prop ?designator (to lift))
                     (desig-prop ?designator (obj ?obj))
                     (desig-prop ?obj (grasp-point ?grasp-point))
                     (desig-prop ?obj (collision-object-name ?collision-object-name)))

  (cram-reasoning:<- (action-desig ?designator (carry ?obj))
                     (desig-prop ?designator (to carry))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (navigation ?goal))
                     (desig-prop ?designator (type navigation))
                     (desig-prop ?designator (goal ?goal)))

  (cram-reasoning:<- (action-desig ?designator (put-down ?obj ?loc ?collision-object-name))
                     (desig-prop ?designator (to put-down))
                     (desig-prop ?designator (obj ?obj))
                     (desig-prop ?designator (at ?loc))
                     (desig-prop ?designator (collision-object-name ?collision-object-name)))

  (cram-reasoning:<- (action-desig ?designator (park ?obj))
                     (desig-prop ?designator (to park))
                     (desig-prop ?designator (obj ?obj))))

(cram-reasoning:def-fact-group manipulation-actions (cram-process-modules:matching-process-module
                                                     cram-process-modules:available-process-module)
  (cram-reasoning:<- (cram-process-modules:matching-process-module ?designator suturo-planning-pm-manipulation)
                     (or (desig-prop ?designator (to follow))
                         (desig-prop ?designator (to grasp))
                         (desig-prop ?designator (to lift))
                         (desig-prop ?designator (to carry))
                         (desig-prop ?designator (type navigation))
                         (desig-prop ?designator (to put-down))
                         (desig-prop ?designator (to park))
                         (desig-prop ?designator (to move-arm-cam)) 
                         (desig-prop ?designator (to move-mast-cam))))
  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-manipulation)))
