(defsystem suturo-planning-planlib
  :depends-on (roslisp
               cram-plan-library
               cram-language
               suturo_interface_msgs-srv)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-plan" :depends-on ("package"))
             (:file "search-objects" :depends-on ("package"))
             (:file "suturo-planning-plan-search-objects" :depends-on("package"))
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
