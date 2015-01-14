(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-perception
  (:nicknames :perception)
  (:use #:desig
        #:cl
        #:cl-transforms)
  (:export #:suturo-planning-pm-perception
           #:get-gripper-perception )
  (:desig-properties
    #:to
    #:color
    #:perceive
    #:obj))
