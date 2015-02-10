(defsystem suturo-planning-constants
  :depends-on (roslisp
               cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-constants-services" :depends-on ("package"))
             (:file "failures" :depends-on ("package"))
             (:file "suturo-planning-constants" :depends-on ("package"))))))
