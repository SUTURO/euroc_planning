(in-package :planlib)

(define-condition map-scanning-failed (simple-plan-failure) ()
  (:default-initargs :format-control "map-scanning-failed"))
(define-condition objects-information-failed (simple-plan-failure) ()
  (:default-initargs :format-control "objects-information-failed"))
(define-condition objects-in-place-failed (simple-plan-failure) ()
  (:default-initargs :format-control "objects-in-place-failed"))
