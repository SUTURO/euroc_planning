(in-package :perception)

(cram-reasoning:def-fact-group perception-action-designator (action-desig)
  
  (cram-reasoning:<- (action-desig ?designator (perceive ?obj))
                     (desig-prop ?designator (to perceive))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (perceive-scene-with-origin ?scenecam ?base-origin))
                     (desig-prop ?designator (to perceive-scene-with))
                     (desig-prop ?designator (scenecam ?scenecam))
                     (desig-prop ?designator (base-origin ?base-origin)))

  (cram-reasoning:<- (action-desig ?designator (perceive-scene-with ?scenecam))
                     (desig-prop ?designator (to perceive-scene-with))
                     (desig-prop ?designator (scenecam ?scenecam))))

(cram-reasoning:def-fact-group perception-actions (cram-process-modules:matching-process-module
                                                   cram-process-modules:available-process-module)
  (cram-reasoning:<- (cram-process-modules:matching-process-module ?designator suturo-planning-pm-perception)
                     (or (desig-prop ?designator (to perceive))
                         (desig-prop ?designator (to perceive-scene-with))))
  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-perception)))
