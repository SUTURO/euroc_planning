(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-manipulation
  (:nicknames :manipulation)
  (:use #:desig
        #:constants
        #:cl)
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
    #:move-mast-cam))
