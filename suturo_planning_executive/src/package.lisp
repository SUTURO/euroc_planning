(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-executive
  (:nicknames :exec)
  (:use #:cpl
        #:cram-plan-library
        #:roslisp
        #:roslisp-utilities
        #:cram-designators
        #:perception
        #:manipulation
        #:suturo-planning-planlib)
  (:export #:main)
  (:desig-properties
    #:type
    #:trajectory
    #:to
    #:grasp
    #:lift
    #:carry
    #:follow
    #:perceive
    #:pose
    #:obj
    #:of
    #:pose
    #:cube))
