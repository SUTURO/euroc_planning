(defsystem suturo-planning-pm-perception
  :depends-on (roslisp
               process-modules
               designators
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-pm-perception" :depends-on ("package"))))))
