(defsystem suturo-planning-pm-manipulation
  :depends-on (roslisp
               process-modules
               designators
               alexandria
               moveit_msgs-msg
               suturo_planning_manipulation-srv)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on("package"))
             (:file "suturo-planning-pm-manipulation" :depends-on ("package" "designators"))))))
