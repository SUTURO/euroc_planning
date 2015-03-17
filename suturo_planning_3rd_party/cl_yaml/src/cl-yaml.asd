(defsystem cl-yaml
  :author "Fernando Borretti <eudoxiahp@gmail.com>"
  :maintainer "Fernando Borretti <eudoxiahp@gmail.com>"
  :license "MIT"
  :version "0.1"
  :depends-on (:cl-libyaml
               :alexandria
               :cl-ppcre
               :parse-number
               :uiop)
  :components ((:module "src"
                :serial t
                :components
                ((:file "error")
                 (:file "float")
                 (:file "scalar")
                 (:file "parser")
                 (:file "emitter")
                 (:file "yaml"))))
  :description "A YAML parser and emitter."
  :in-order-to ((test-op (test-op cl-yaml-test))))
