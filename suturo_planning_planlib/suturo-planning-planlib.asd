(defsystem suturo-planning-planlib
  :depends-on (roslisp
               designators
               cram-plan-library
               cram-plan-failures
               cram-language
               cl-utilities
               cram-reasoning
               suturo-planning-environment
               geometry_msgs-msg
               suturo_interface_msgs-srv
               suturo_interface_msgs-msg
               suturo_perception_msgs-srv
               suturo_perception_msgs-msg
               suturo_manipulation_msgs-srv
               suturo_manipulation_msgs-msg
               suturo-planning-pm-manipulation
               suturo-planning-pm-perception
               suturo-planning-constants
               suturo_environment_msgs-srv)              
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-plan" :depends-on ("package"))
             (:file "search-objects" :depends-on ("package"))
             (:file "suturo-planning-plan-scan-map-mast-cam" :depends-on ("package"))
             (:file "suturo-planning-plan-search-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-init" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-scan-obstacles" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-classify-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-pose-estimate-object" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-focus-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-clean-up-plan" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-choose-object" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-utils" :depends-on("package" "suturo-planning-plan"))
             (:file "goals" :depends-on ("package" "suturo-planning-plan-scan-map-mast-cam"))
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
