(in-package :manipulation)

(cram-reasoning:def-fact-group manipulation-action-designator (action-desig)
  (cram-reasoning:<- (action-desig ?designator (follow ?pose))
                     (desig-prop ?designator (to follow))
                     (desig-prop ?designator (pose ?pose)))

  (cram-reasoning:<- (action-desig ?designator (perceive ?obj))
                     (desig-prop ?designator (to grasp))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (lift ?obj))
                     (desig-prop ?designator (to lift))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (carry ?obj))
                     (desig-prop ?designator (to carry))
                     (desig-prop ?designator (obj ?obj))))

(cram-reasoning:def-fact-group manipulation-actions (cram-process-modules:matching-process-module
                                                     cram-process-modules:available-process-module)
  (cram-reasoning:<- (cram-process-modules:matching-process-module ?designator suturo-planning-pm-manipulation)
                     (or (desig-prop ?designator (to follow))
                         (desig-prop ?designator (to grasp))
                         (desig-prop ?designator (to lift))
                         (desig-prop ?designator (to carry))))
  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-manipulation)))
