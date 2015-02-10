(defsystem suturo-planning-executive
  :depends-on (roslisp
               roslisp-utilities
               cram-language
               cram-plan-library
               cram-reasoning
               process-modules
               suturo-planning-constants
               suturo-planning-environment
               suturo-planning-pm-perception
               suturo-planning-pm-manipulation
               suturo-planning-planlib)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "belief" :depends-on ("package"))
             (:file "suturo-planning-executive" :depends-on ("package" "belief"))))))
