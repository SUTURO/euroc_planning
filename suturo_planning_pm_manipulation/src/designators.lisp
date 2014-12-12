(in-package :manipulation)

(cram-reasoning:def-fact-group manipulation-action-designator (action-desig)
  (cram-reasoning:<- (action-desig ?designator (follow ?pose))
                     (desig-prop ?designator (to follow))
                     (desig-prop ?designator (pose ?pose)))

  (cram-reasoning:<- (action-desig ?designator (grasp ?obj))
                     (desig-prop ?designator (to grasp))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (lift ?obj))
                     (desig-prop ?designator (to lift))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (carry ?obj))
                     (desig-prop ?designator (to carry))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (navigation ?goal))
                     (desig-prop ?designator (type navigation))
                     (desig-prop ?designator (goal ?goal)))

  (cram-reasoning:<- (action-desig ?designator (put-down ?obj ?loc))
                     (desig-prop ?designator (to put-down))
                     (desig-prop ?designator (obj ?obj))
                     (desig-prop ?designator (at ?loc)))

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
                         (desig-prop ?designator (to park))))
  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-manipulation)))
