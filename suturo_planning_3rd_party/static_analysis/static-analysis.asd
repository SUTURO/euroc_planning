(defsystem static-analysis
  :depends-on ( iterate
                alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "static-analysis" :depends-on ("package"))))))
