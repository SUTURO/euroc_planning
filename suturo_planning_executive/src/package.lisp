(in-package :cl-user)

(desig-props:def-desig-package suturo-planning-executive
  (:nicknames :exec)
  (:use #:cpl
        #:roslisp
        #:roslisp-utilities
        #:cram-designators
        #:perception
        #:manipulation
        #:suturo-planning-planlib)
  (:export #:main))
