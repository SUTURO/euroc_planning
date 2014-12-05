(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-executive
  (:nicknames :exec)
  (:use #:cpl
        #:cpl-desig-supp
        #:cram-plan-library
        #:cram-roslisp-common
        #:roslisp
        #:cram-designators
        #:perception
        #:manipulation
        #:suturo-planning-planlib)
  (:export #:main)
  (:desig-properties
    #:type
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
    #:cube))
