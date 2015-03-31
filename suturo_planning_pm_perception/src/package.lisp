(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-perception
  (:nicknames :perception)
  (:use #:desig
        #:cl
        #:cl-transforms
        #:cpl-impl
        #:constants)
  (:export #:suturo-planning-pm-perception)
  (:desig-properties
    #:base-origin
    #:classify-object
    #:color
    #:find-objects-in-map
    #:focus-object
    #:get-gripper-perception
    #:ids 
    #:obj
    #:objects
    #:perceive
    #:perceive-scene-with
    #:pose-estimate-object
    #:scenecam
    #:to)
  (:documentation
   "This package defines the process module suturo-planning-pm-manipulation and contains actions that are concerned to the manipulation part of a plan. To define new actions you have to do 
the following steps:
- Define an action-handler and implement the functionality
- Add the reasoning for the action-handler / the process module in designators.lisp
- Add the used designator properties to the desig-props in this package.lisp
- Add the used designator properties to the desig-props of the calling package
"
))
