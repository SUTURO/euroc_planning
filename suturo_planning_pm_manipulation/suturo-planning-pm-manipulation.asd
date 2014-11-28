(defsystem suturo-planning-pm-manipulation
  :depends-on (roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-pm-manipulation" :depends-on ("package"))))))
