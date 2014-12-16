(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-environment
  (:nicknames :environment)
  (:use #:cl
        #:roslisp
        #:roslisp-utilities
        #:desig)
  (:export #:get-model-pose)
  (:desig-properties
    #:at
    #:obj
    #:obj-pos
    #:pose
    #:of))
