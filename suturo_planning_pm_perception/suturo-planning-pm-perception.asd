(defsystem suturo-planning-pm-perception
  :depends-on (roslisp 
               suturo_perception_msgs-srv
               suturo_perception_msgs-msg
               suturo_interface_msgs-srv
               process-modules
               designators
               alexandria
               cl-transforms)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on ("package"))
             (:file "recognized-object" :depends-on ("package"))
             (:file "search-objects" :depends-on ("package"))
             (:file "suturo-planning-pm-perception" :depends-on ("package" "designators"))))))
