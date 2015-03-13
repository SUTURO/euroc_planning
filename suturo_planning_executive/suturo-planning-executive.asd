(defsystem suturo-planning-executive
  :serial t
  :depends-on (roslisp
               roslisp-utilities
               cram-language
               cram-plan-library
		       cram-plan-failures
               cram-reasoning
               process-modules
               suturo_msgs-msg
               suturo-planning-constants
               suturo-planning-environment
               suturo-planning-pm-perception
               suturo-planning-pm-manipulation
               suturo-planning-planlib
               cram-beliefstate
               euroc_c2_msgs-srv
               euroc_c2_msgs-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "belief" :depends-on ("package"))
             (:file "suturo-planning-executive" :depends-on ("package" "belief"))))))
