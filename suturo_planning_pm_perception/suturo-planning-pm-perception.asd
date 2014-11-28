(defsystem suturo-planning-pm-perception
  :depends-on (roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-pm-perception" :depends-on ("package"))))))
