(defsystem suturo-planning-pm-manipulation
  :depends-on (roslisp
               cram-plan-library
               cram-language
               process-modules
               designators
               alexandria
               suturo_interface_msgs-srv
               suturo_interface_msgs-msg
               suturo_perception_msgs-srv
               suturo-planning-constants
               moveit_msgs-msg
               suturo_planning_manipulation-srv)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "constants" :depends-on("package"))
             (:file "designators" :depends-on("package"))
             (:file "manipulation-utils" :depends-on("package"))
             (:file "suturo-planning-pm-manipulation" :depends-on ("package" "designators"))
             (:file "suturo-planning-pm-manipulation-scan-map" :depends-on ("package" "designators" "suturo-planning-pm-manipulation"))))))
