(in-package :perception)

(cram-reasoning:def-fact-group perception-action-designator (action-desig)
  
  (cram-reasoning:<- (action-desig ?designator (perceive ?obj))
    (desig-prop ?designator (to perceive))
    (desig-prop ?designator (obj ?obj)))
  
  (cram-reasoning:<- (action-desig ?designator (perceive-scene-with ?cam))
    (desig-prop ?designator (to perceive-scene-with))
    (desig-prop ?desginator (scenecam ?cam))))

(cram-reasoning:def-fact-group perception-actions (cram-process-modules:matching-process-module
                                                   cram-process-modules:available-process-module)
  (cram-reasoning:<- (cram-process-modules:matching-process-module ?designator suturo-planning-pm-perception)
                     (or (desig-prop ?designator (to perceive))
                         (desig-prop ?desginator (to perceive-scene-with))))

  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-perception)))
