(defsystem suturo-planning-pm-perception
<<<<<<< HEAD
  :depends-on (roslisp suturo_perception_msgs-srv)
=======
  :depends-on (roslisp
               process-modules
               designators
               alexandria)
>>>>>>> abc35351d98a3877da41cc663fc5c2d665ca6ea4
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on ("package"))
             (:file "suturo-planning-pm-perception" :depends-on ("package" "designators"))))))
