(defsystem suturo-planning-environment
  :depends-on (roslisp
               roslisp-utilities
               cram-language
               gazebo_msgs-srv
               gazebo_msgs-msg
               trajectory_msgs-msg
               designators-ros
               designators
               cl-tf
               cl-transforms)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "gazebo" :depends-on ("package"))
             (:file "suturo-planning-environment" :depends-on ("package" "gazebo"))
             (:file "target-zones" :depends-on ("package"))))))
