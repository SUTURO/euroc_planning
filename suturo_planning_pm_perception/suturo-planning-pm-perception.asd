(defsystem suturo-planning-pm-perception
  :depends-on (roslisp 
	       suturo_perception_msgs-srv
               process-modules
               designators
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on ("package"))
             (:file "suturo-planning-pm-perception" :depends-on ("package" "designators"))))))
