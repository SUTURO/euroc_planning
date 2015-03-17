(defsystem cl-libyaml
  :author "Fernando Borretti <eudoxiahp@gmail.com>"
  :maintainer "Fernando Borretti <eudoxiahp@gmail.com>"
  :license "MIT"
  :version "0.1"
  :depends-on (:cffi)
  :components ((:module "src"
                :serial t
                :components
                ((:file "library")
                 (:file "version")
                 (:file "util")
                 (:file "basic-types")
                 (:file "style")
                 (:file "node")
                 (:file "token")
                 (:file "event")
                 (:file "document")
                 (:file "parser")
                 (:file "macros"))))
  :description "A binding to the libyaml library."
  :in-order-to ((test-op (test-op cl-libyaml-test))))
