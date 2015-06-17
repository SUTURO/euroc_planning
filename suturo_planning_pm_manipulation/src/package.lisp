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
    #:prefer-grasp-position
    #:open-gripper
    #:position
    #:collision-object)
  (:documentation
   "This package defines the process module suturo-planning-pm-manipulation and contains actions that are concerned to the manipulation part of a plan. To define new actions you have to do 
the following steps:
- Define an action-handler and implement the functionality
- Add the reasoning for the action-handler / the process module in designators.lisp
- Add the used designator properties to the desig-props in this package.lisp
- Add the used designator properties to the desig-props of the calling package
"
))
