(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-pm-manipulation
  (:nicknames :manipulation)
  (:use #:desig
        #:cl)
  (:export #:suturo-planning-pm-manipulation)
  (:desig-properties
    #:to
    #:follow
    #:grasp
    #:lift
    #:carry
    #:pose
    #:obj))
