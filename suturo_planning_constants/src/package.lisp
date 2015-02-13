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
           #:+service-name-create-poses-for-object-scanning+
	   
           #:+topic-name-get-yaml+
           
           #:+base-origin-topic+ 
           #:+timeout-service+
           ; ^^^ Constants ^^^
           ; vvv Failures vvv
           #:map-scanning-failed
           #:moving-mast-cam-failed
           #:moving-arm-failed
           #:objects-information-failed
           #:objects-in-place-failed

	   #:+suturo-planning-constants-defined+
	   #:+scan-obstacles-angle+
	   #:+scan-obstacles-distance-parameter-factor+
	   #:+scan-obstacles-distance-parameter-offset+
	   #:+scan-obstacles-number-of-poses+
	   #:+scan-obstacles-arm-max-distance+))



