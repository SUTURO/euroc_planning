(defsystem suturo-planning-pm-manipulation
  :depends-on (roslisp
               process-modules
               designators
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on("package"))
             (:file "suturo-planning-pm-manipulation" :depends-on ("package" "designators"))
             (:file "suturo-planning-pm-manipulation-scan-map" :depends-on ("package" "designators"))))))
