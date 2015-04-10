(print "lolz")
(defconstant package-names '("suturo-planning-executive" 
                             "suturo-planning-constants"
                             "suturo-planning-environment"
                             "suturo-planning-pm-manipulation"
                             "suturo-planning-pm-perception"))
(defconstant name-doc-folder "/doc/")
(ros-load:load-system "clod" "clod")

(defun split-string-on-char(string-to-split char-to-split-on)
           (let ((last-found-char))
             (loop for i from 0 to (- (length string-to-split) 1) do
               (if (string= (char string-to-split i) char-to-split-on)
                   (setf last-found-char i)))
             (print last-found-char)
             (subseq string-to-split 0 last-found-char)))

(defun document-packages ()
    (loop for name in package-names do
      (multiple-value-bind (found found-system pathname previous previous-time) (asdf:locate-system name)
        (if found
              (clod:document-package name (concatenate 'string (split-string-on-char (namestring pathname) "/") name-doc-folder name ".org"))))))

(document-packages)
