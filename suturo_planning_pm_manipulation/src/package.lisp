(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-manipulation
  (:nicknames :manipulation)
  (:use #:desig
        #:cl
        #:cl-transforms
        #:roslisp
        #:constants)
  (:export #:suturo-planning-pm-manipulation
           #:scan-map
           #:scan-map-action)
  (:desig-properties
    #:to
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
    #:pose
    #:obj
    #:pan
    #:tilt
    #:move-arm-cam
    #:pose-name
    #:move-mast-cam))
