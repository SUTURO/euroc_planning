;;;;  -*- mode: emacs-lisp; coding: iso-latin-1-unix -*-

;;; This code can be added to your .emacs file to allow editing of docstrings
;;; using org mode, WITHIN lisp mode.
;;; See the CLOD documentation for details.

;; First, add doc string fontification for some common lisp forms that
;; lisp-mode.el omits.
;; Can't do DEFMETHOD docstrings as defmethod form may contain :BEFORE,
;; :AFTER etc.
(put 'defparameter   'doc-string-elt 3)
(put 'defconstant   'doc-string-elt 3)
(put 'deftype   'doc-string-elt 3)

;;; Change this to the directory whwere nXhtml is installed.
(add-to-list 'load-path (mkfilename *user-emacs-root* "/mmm-mode"))

(require 'mmm-auto)
(setq mmm-global-mode 'maybe)
(mmm-add-mode-ext-class 'lisp-mode  nil  'org-submode)
(mmm-add-mode-ext-class 'slime-mode  nil  'org-submode)
;; The above, using major mode symbols, didn't seem to work for me so I
;; also added this line (regexp uses same format as major-mode-alist):
(mmm-add-mode-ext-class nil  "\\.lisp$"  'org-submode)
(mmm-add-mode-ext-class nil  "\\.lsp$"  'org-submode)
(mmm-add-mode-ext-class nil  "\\.asd$"  'org-submode)
(setq mmm-submode-decoration-level 2)

;; This prevents transient loss of fontification on first
;; calling `mmm-ify-by-class'
(defadvice mmm-ify-by-class (after refontify-after-mmm activate compile)
  (font-lock-fontify-buffer))

;; Control-backquote "refreshes" MMM-mode in current buffer.
(global-set-key [?\C-`] (lambda () (interactive) (mmm-ify-by-class 'org-submode)))

;; And the definition of 'org-submode
(mmm-add-group 'org-submode
               '((org-submode-1
                  :submode org-mode
                  :face mmm-declaration-submode-face
                  ;; Match '(:documentation "...")' docstrings
                  :front "(:documentation[\t\n ]+\""
                  ;; The following is a regexp, to match the end of a
                  ;; string. Tolerates an EVEN number of backslashes
                  ;; preceding the quote.
                  :back "[^\\]\\(\\\\\\\\\\)*\"" ;"[^\\]\""
                  :back-offset 1
                  :end-not-begin t)
                 (org-submode-2
                  :submode org-mode
                  :face mmm-declaration-submode-face
                  :front "[^\\]\""
                  :back "[^\\]\\(\\\\\\\\\\)*\"" ;"[^\\]\""
                  :back-offset 1
                  :front-verify check-docstring-match
                  :end-not-begin t)))

(defun face-at (pos)
  (save-excursion
    (goto-char pos)
    (face-at-point)))


(defun check-docstring-match ()
  (interactive)
  (let ((beg (match-beginning 0))
        (end (match-end 0)))
  (cond
   ;; If the character just BEFORE the match is in string face,
   ;; the match will not be a doc string
   ;((member (face-at beg) '(font-lock-doc-face font-lock-string-face))
   ; nil)
   ;; Docstring if emacs has fontified it in 'docstring' face
   ((eql (face-at end) 'font-lock-doc-face)
    t)
   ;; Docstring if the first three characters after the opening
   ;; quote are "###"
   ((string= (buffer-substring end (+ 3 end)) "###")
    t)
   (t nil))))
