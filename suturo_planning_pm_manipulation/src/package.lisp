(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-manipulation
  (:nicknames :manipulation)
  (:use #:desig
        #:constants
        #:cram-plan-library
        #:cram-plan-failures
        #:roslisp
        #:cl-transforms
        #:cl)
  (:export #:suturo-planning-pm-manipulation
           #:scan-map
           #:*base-origin*
					 #:*object-designator*
           #:init 
           #:call-add-collision-objects
           #:scan-map-action)
  (:desig-properties
    #:to
    #:pose
    #:type
    #:navigation
    #:goal
    #:put-down
    #:at
    #:follow
    #:grasp
    #:lift
    #:carry
    #:park
    #:obj
    #:pan
    #:do-now-blow-up-list
    #:tilt
    #:move-arm-cam
    #:pose-name
    #:move-mast-cam
    #:grasp-position
    #:grasp-point
    #:collision-object))
