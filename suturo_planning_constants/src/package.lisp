(in-package :cl-user)

(defpackage suturo-planning-constants
  (:nicknames :constants)
  (:use #:roslisp
        #:cl)
  (:export #:+suturo-planning-constants-services-defined+
           #:+service-name-move-mastcam+
           #:+service-name-add-point-cloud+
           #:+service-name-move-robot+
           #:+service-name-get-base-origin+
           #:+base-origin-topic+ 
           #:+timeout-service+))

