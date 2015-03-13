(cl:defpackage :static-analysis
  (:use :cl :cl-user :iterate)
  (:export #:function-call-graph
           #:->dot
           #:call-graph->dot))
