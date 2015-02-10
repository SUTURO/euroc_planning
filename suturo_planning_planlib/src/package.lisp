;(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-planlib
  (:nicknames :planlib)
  (:use #:roslisp
        #:cpl
        #:cram-plan-library
        #:constants
        #:cram-designators)
  (:export #:do-planning
           #:*current-state*
           #:*current-transition*
           #:*taskdata*
           #:map-scanned
           #:objects-informed
           #:unknown-scanned
           #:object-classified
           #:pose-estimated
           #:objects-in-place
           #:map-scanning-failed
           #:objects-information-failed
           #:objects-in-place-failed)
  (:desig-properties
   #:to 
   #:pan
   #:tilt
   #:scenecam
   #:base-origin
   #:perceive-scene-with
   #:classify-object
   #:focus-object
   #:pose-estimate-object
   #:move-mast-cam
   #:pose-name
   #:obj
   #:move-arm-cam))
