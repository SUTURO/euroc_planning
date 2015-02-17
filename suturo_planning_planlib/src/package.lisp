;(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-planlib
  (:nicknames :planlib)
  (:use #:roslisp
        #:environment
        #:cpl
        #:cram-plan-library
        #:cram-plan-failures
        #:cram-language
        #:constants
        #:cram-designators
        #:cl-transforms
        #:cram-language-designator-support)
  (:export #:do-planning
           #:*current-state*
           #:*current-transition*
           #:*taskdata*
           #:map-scanned
           #:objects-informed
           #:unknown-scanned
           #:object-classified
           #:pose-estimated
           #:objects-in-place)
  (:desig-properties
   #:to 
   #:pose-estimate-object
   #:id
   #:pan
   #:tilt
   #:at
   #:scenecam
   #:base-origin
   #:perceive-scene-with
   #:classify-object
   #:focus-object
   #:pose-estimate-object
   #:move-mast-cam
   #:pose-name
   #:expected-object
   #:pose
   #:type
   #:max-distance
   #:obj
   #:gripper
   #:arm
   #:move-arm-cam))
