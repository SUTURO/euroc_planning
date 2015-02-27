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
  (:export            
           #:map-scanned
           #:objects-informed
           #:unknown-scanned
           #:object-classified
           #:pose-estimated
           #:objects-in-place)
  (:desig-properties
   #:arm
   #:at
   #:base-origin
   #:classify-object
   #:do-not-blow-up-list
   #:expected-object
   #:find-objects-in-map
   #:focus-object
   #:get-gripper-perception
   #:gripper
   #:ids
   #:max-distance
   #:move-arm-cam
   #:move-mast-cam
   #:obj
   #:objects
   #:pan
   #:perceive-scene-with
   #:pose-estimate-object
   #:pose
   #:pose-name
   #:scenecam
   #:to 
   #:tilt
   #:type))
