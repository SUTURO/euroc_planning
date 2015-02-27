(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-executive
  (:nicknames :exec)
  (:use #:cpl
        #:cpl-desig-supp
        #:cram-plan-library
        #:cram-roslisp-common
        #:roslisp
        #:cram-designators
        #:constants
        #:perception
        #:manipulation
        #:constants
        #:suturo-planning-planlib)
  (:export #:task-selector #:task1)
  (:desig-properties
    #:type
    #:max-distance
    #:expected-object
    #:trajectory
    #:navigation
    #:to
    #:grasp
    #:lift
    #:carry
    #:follow
    #:perceive
    #:pose
    #:obj
    #:gripper
    #:of
    #:at
    #:pose
    #:cube
    #:scenecam
    #:collision-object
    #:perceive-scene-with
    #:move-mast-cam
    #:move-arm-cam
    #:pose-name
    #:classify-object
    #:focus-object
    #:objects
    #:find-objects-in-map
    #:pose-estimate-object
    #:scan-map))
