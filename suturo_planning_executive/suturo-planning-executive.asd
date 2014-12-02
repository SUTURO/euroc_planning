(defsystem suturo-planning-executive
  :depends-on (roslisp
               roslisp-utilities
               cram-language
               process-modules
               suturo-planning-environment
               suturo-planning-pm-perception
               suturo-planning-pm-manipulation
               suturo-planning-planlib)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-executive" :depends-on ("package"))))))
