(defsystem suturo-planning-environment
  :depends-on (roslisp
               roslisp-utilities)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-environment" :depends-on ("package"))))))
