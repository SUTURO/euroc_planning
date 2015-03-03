(defsystem suturo-planning-pm-manipulation
  :depends-on (roslisp
               cram-plan-library
               cram-plan-failures
               cram-language
               process-modules
               designators
               alexandria
               moveit_msgs-msg
               suturo_manipulation_msgs-msg
               suturo_manipulation_msgs-srv
               suturo_perception_msgs-msg
               suturo_perception_msgs-srv
               suturo_interface_msgs-msg
               suturo_interface_msgs-srv
               suturo-planning-constants
               suturo-planning-environment)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "constants" :depends-on("package"))
             (:file "designators" :depends-on("package"))
             (:file "manipulation-utils" :depends-on("package" "constants"))
             (:file "suturo-planning-pm-manipulation" :depends-on ("package" "designators" "manipulation-utils"))
             (:file "suturo-planning-pm-manipulation-scan-map" :depends-on ("package" "designators" "suturo-planning-pm-manipulation"))))))
