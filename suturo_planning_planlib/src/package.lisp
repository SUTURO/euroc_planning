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
           #:map-scanning-failed
           #:objects-information-failed
           #:objects-in-place-failed)
  (:desig-properties
   #:to 
   #:pan
   #:tilt
   #:scenecam
   #:perceive-scene-with
   #:move-mast-cam))
