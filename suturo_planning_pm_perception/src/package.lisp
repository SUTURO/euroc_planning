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
    #:to))
