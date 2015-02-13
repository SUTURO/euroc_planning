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
           #:+service-name-euroc-object-to-odom-combined+ 
           #:+service-name-classify-objects+

           #:+topic-name-get-yaml+
           
           #:+base-origin-topic+ 
           #:+timeout-service+))



