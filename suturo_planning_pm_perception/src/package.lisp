(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-perception
  (:nicknames :perception)
  (:use #:desig
        #:cl
        #:cl-transforms
        #:cpl-impl
        #:constants)
  (:export #:suturo-planning-pm-perception
           #:get-gripper-perception)
  (:desig-properties
    #:to
    #:color
    #:perceive
    #:perceive-scene-with
    #:classify-object
    #:focus-object
    #:pose-estimate-object
    #:scenecam
    #:base-origin
    #:pose-estimate-object
    #:id
    #:obj))
