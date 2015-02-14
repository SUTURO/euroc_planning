(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-environment
  (:nicknames :environment)
  (:use #:cl
        #:roslisp
        #:roslisp-utilities
        #:desig)
  (:export #:get-model-pose
           #:get-target-zones
           #:find-matching-target-zone)
  (:desig-properties
    #:at
    #:type
    #:expected-object
    #:max-distance
    #:obj
    #:obj-pos
    #:pose
    #:of))
