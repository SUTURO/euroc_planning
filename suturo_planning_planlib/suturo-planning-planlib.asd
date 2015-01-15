(defsystem suturo-planning-planlib
  :depends-on (roslisp
               cram-plan-library
               cram-language
               cl-utilities
               suturo_interface_msgs-srv
               suturo_interface_msgs-msg
               suturo_planning_manipulation-srv
               suturo_perception_msgs-srv
               suturo-planning-pm-perception)              
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
             (:file "suturo-planning-plan-classify-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-pose-estimate-object" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
