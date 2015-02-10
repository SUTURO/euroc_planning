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
           #:+timeout-service+
           ; ^^^ Constants ^^^
           ; vvv Failures vvv
           #:map-scanning-failed
           #:moving-mast-cam-failed
           #:moving-arm-failed
           #:objects-information-failed
           #:objects-in-place-failed))

