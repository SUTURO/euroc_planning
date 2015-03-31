(defsystem suturo-planning-planlib
  :serial t
  :depends-on (roslisp
               designators
               cram-plan-library
               cram-plan-failures
               cram-language
               cl-utilities
               cram-reasoning
               geometry_msgs-msg
               suturo_perception_msgs-msg
               suturo_perception_msgs-srv
               suturo_manipulation_msgs-srv
               suturo_startup_msgs-msg
               suturo_startup_msgs-srv
               suturo-planning-environment
               suturo-planning-pm-manipulation
               suturo-planning-pm-perception
               suturo-planning-constants
               suturo_environment_msgs-srv)              
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-plan" :depends-on ("package"))
             (:file "suturo-planning-plan-scan-map-mast-cam" :depends-on ("package"))
             (:file "suturo-planning-plan-utils" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-scan-obstacles" :depends-on("package" "suturo-planning-plan"))
             (:file "goals" :depends-on ("package" "suturo-planning-plan-scan-map-mast-cam"))))))
