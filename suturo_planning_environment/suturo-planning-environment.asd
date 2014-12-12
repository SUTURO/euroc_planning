(defsystem suturo-planning-environment
  :depends-on (roslisp
               roslisp-utilities
               designators-ros
               designators
               cl-tf
               cl-transforms)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-environment" :depends-on ("package"))))))
