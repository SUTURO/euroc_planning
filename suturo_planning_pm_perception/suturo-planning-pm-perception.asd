(defsystem suturo-planning-pm-perception
  :depends-on (roslisp suturo_perception_msgs-srv)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-pm-perception" :depends-on ("package"))))))
