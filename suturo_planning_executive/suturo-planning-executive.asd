(defsystem suturo-planning-executive
  :depends-on (roslisp
               roslisp-utilities
               cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-executive" :depends-on ("package"))))))
