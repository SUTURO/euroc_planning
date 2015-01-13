(defsystem suturo-planning-planlib
  :depends-on (roslisp
               cram-plan-library
               cram-language
               cl-utilities
               suturo_interface_msgs-srv
               suturo_interface_msgs-msg
               suturo_planning_manipulation-srv)              
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-plan" :depends-on ("package"))
             (:file "search-objects" :depends-on ("package"))
             (:file "suturo-planning-plan-search-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-scan-map-mast-cam" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-scan-shadow" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-init" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
