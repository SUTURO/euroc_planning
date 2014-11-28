(defsystem suturo-planning-executive
  :depends-on (roslisp
               cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "suturo-planning-executive" :depends-on ("package"))))))
