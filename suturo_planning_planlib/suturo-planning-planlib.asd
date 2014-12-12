(defsystem suturo-planning-planlib
  :depends-on (roslisp
               cram-plan-library)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
