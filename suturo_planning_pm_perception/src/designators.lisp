(in-package :perception)

(cram-reasoning:def-fact-group perception-action-designator (action-desig)
  
  (cram-reasoning:<- (action-desig ?designator (perceive ?obj))
                     (desig-prop ?designator (to perceive))
                     (desig-prop ?designator (obj ?obj)))

  (cram-reasoning:<- (action-desig ?designator (pose-estimate-object ?ids))
                     (desig-prop ?designator (to pose-estimate-object))
                     (desig-prop ?designator (ids ?ids)))

  (cram-reasoning:<- (action-desig ?designator (find-objects-in-map ?objects))
                     (desig-prop ?designator (to find-objects-in-map))
                     (desig-prop ?designator (objects ?objects)))

  (cram-reasoning:<- (action-desig ?designator (get-gripper-perception))
                     (desig-prop ?designator (to get-gripper-perception)))

  (cram-reasoning:<- (action-desig ?designator (perceive-scene-with-origin ?scenecam ?base-origin))
                     (desig-prop ?designator (to perceive-scene-with))
                     (desig-prop ?designator (scenecam ?scenecam))
                     (desig-prop ?designator (base-origin ?base-origin)))

  (cram-reasoning:<- (action-desig ?designator (perceive-scene-with ?scenecam))
                     (desig-prop ?designator (to perceive-scene-with))
                     (desig-prop ?designator (scenecam ?scenecam)))
  
  (cram-reasoning:<- (action-desig ?designator (classify-object ?object))
                     (desig-prop ?designator (to classify-object))
                     (desig-prop ?designator (obj ?object)))
  
  (cram-reasoning:<- (action-desig ?designator (focus-object ?object))
                     (desig-prop ?designator (to focus-object))
                     (desig-prop ?designator (obj ?object)))
  
  (cram-reasoning:<- (action-desig ?designator (pose-estimate-object ?object))
                     (desig-prop ?designator (to pose-estimate-object))
                     (desig-prop ?designator (obj ?object))))

(cram-reasoning:def-fact-group perception-actions (cram-process-modules:matching-process-module
                                                   cram-process-modules:available-process-module)
  (cram-reasoning:<- (cram-process-modules:matching-process-module ?designator suturo-planning-pm-perception)
                     (or (desig-prop ?designator (to perceive))
                         (desig-prop ?designator (to perceive-scene-with))
                         (desig-prop ?designator (to classify-object))
                         (desig-prop ?designator (to get-gripper-perception))
                         (desig-prop ?designator (to focus-object))
                         (desig-prop ?designator (to find-objects-in-map))
                         (desig-prop ?designator (to pose-estimate-object))))
  (cram-reasoning:<- (cram-process-modules:available-process-module suturo-planning-pm-perception)))
