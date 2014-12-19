(defsystem suturo-planning-planlib
  :depends-on (roslisp
               cram-plan-library
               suturo_interface_msgs-srv)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-search-objects" :depends-on ("package"))
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
