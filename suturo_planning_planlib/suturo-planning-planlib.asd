(defsystem suturo-planning-planlib
  :depends-on (roslisp
               designators
               cram-plan-library
               cram-language
               cl-utilities
               cram-reasoning
               suturo_interface_msgs-srv
               suturo_interface_msgs-msg
               suturo_planning_manipulation-srv
               suturo_perception_msgs-srv
               suturo_perception_msgs-msg
               suturo-planning-pm-manipulation
               suturo-planning-pm-perception
               suturo-planning-constants
               std_msgs-msg)              
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-plan" :depends-on ("package"))
             (:file "search-objects" :depends-on ("package"))
             (:file "suturo-planning-plan-scan-map-mast-cam" :depends-on ("package"))
             (:file "suturo-planning-plan-search-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-init" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-find-objects-in-map" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-classify-objects" :depends-on("package" "suturo-planning-plan" "suturo-planning-find-objects-in-map"))
             (:file "suturo-planning-plan-pose-estimate-object" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-focus-objects" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-clean-up-plan" :depends-on("package" "suturo-planning-plan"))
             (:file "suturo-planning-plan-choose-object" :depends-on("package" "suturo-planning-plan"))
             (:file "goals" :depends-on ("package" "suturo-planning-plan-scan-map-mast-cam"))
             (:file "suturo-planning-planlib" :depends-on ("package"))))))
