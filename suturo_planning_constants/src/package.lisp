(in-package :cl-user)

(defpackage suturo-planning-constants
  (:nicknames :constants)
  (:use #:roslisp
        #:cl)
  (:export #:+suturo-planning-constants-services-defined+
	       #:+scan-obstacles-angle+
	       #:+scan-obstacles-arm-max-distance+
	       #:+scan-obstacles-distance-parameter-factor+
	       #:+scan-obstacles-distance-parameter-offset+
	       #:+scan-obstacles-number-of-poses+
           #:+service-name-add-collision-objects+
           #:+service-name-add-point-cloud+
           #:+service-name-classify-objects+
           #:+service-name-close-gripper+
           #:+service-name-create-poses-for-object-scanning+
           #:+service-name-euroc-object-to-odom-combined+ 
           #:+service-name-grasp-object+
           #:+service-name-get-base-origin+
           #:+service-name-get-map+
           #:+service-name-get-obstacle-regions+
           #:+service-name-move-mastcam+
           #:+service-name-move-robot+
	       #:+service-name-open-gripper+

           #:+topic-name-get-yaml+
           #:+base-origin-topic+
           ; ^^^ Constants ^^^
           ; vvv Failures vvv
           #:map-scanning-failed
           #:moving-mast-cam-failed
           #:moving-arm-failed
           #:objects-information-failed
           #:objects-in-place-failed
           #:+timeout-service+))
