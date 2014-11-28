(defsystem suturo-planning-planlib
  :depends-on (roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
