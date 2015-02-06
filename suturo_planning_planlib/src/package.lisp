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
           #:map-scanned)
  (:desig-properties
   #:to 
   #:pan
   #:tilt
   #:scenecam
   #:base-origin
   #:perceive-scene-with
   #:move-arm-cam
   #:pose-name
   #:move-mast-cam))
