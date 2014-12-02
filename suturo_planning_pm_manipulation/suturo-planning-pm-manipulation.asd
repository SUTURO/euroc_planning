(defsystem suturo-planning-pm-manipulation
  :depends-on (roslisp
               process-modules
               designators
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-pm-manipulation" :depends-on ("package"))))))
