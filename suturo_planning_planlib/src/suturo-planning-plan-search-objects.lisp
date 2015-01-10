(in-package :planlib)

;(def-cram-function init-search-object()
;  (whenever ((pulsed ))
;            (when (eq (value f1:done))
;              (format t "blub"))))


(def-cram-function init-search-object2 ()
  (wait-for (fl-and (eql (fl-funcall #'value *current-state*) "init")
                    (eql (fl-funcall #'value *current-transition*) "start")))
  (format t "done"))


(defun compare-string-fluent(str1 str2)
  (let ((fl (make-fluent :name :local-fluent :value nil)))
    (if (string= str1 str2) 
        (setf fl 1)
        (setf fl 2))))
