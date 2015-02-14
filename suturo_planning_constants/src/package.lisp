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
           #:+service-name-add-collision-objects+
           #:+service-name-get-obstacle-regions+
           #:+service-name-get-map+

           #:+topic-name-get-yaml+
           #:+base-origin-topic+ 

           #:+timeout-service+
           #:map-scanning-failed
           #:moving-mast-cam-failed
           #:moving-arm-failed
           #:objects-information-failed
           #:objects-in-place-failed))
