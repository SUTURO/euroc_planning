;;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*- ;;;;;;;;;;;;;;;;;80
;;;;
;;;;    This file is part of CLOD, by Paul Sexton
;;;;    Released under the Gnu Public License version 3
;;;;
;;;;    CLOD is free software: you can redistribute it and/or modify
;;;;    it under the terms of the GNU General Public License as published by
;;;;    the Free Software Foundation, either version 3 of the License, or
;;;;    (at your option) any later version.
;;;;
;;;;    CLOD is distributed in the hope that it will be useful,
;;;;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;;;;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;;;    GNU General Public License for more details.
;;;;
;;;;    You should have received a copy of the GNU General Public License
;;;;    along with CLOD.  If not, see <http://www.gnu.org/licenses/>.
;;;;
;;;; todo:
;;;; * would be nice to get CL symbols to link to hyperspec
;;;;    (if (eql (symbol-package sym) (find-package :cl)) ...)
;;;;   Currently [[hs:sym]]  --> HyperSpecRoot/sym but this will not work as
;;;;   hyperspec pages are unfortunately not named according to the symbol they
;;;;   describe.
;;;; * table of slots at beginning of each class def
;;;; * reverse auto-doc:
;;;;   CLOD annotates org file to 'mark up' doc strings for each
;;;;   type of entity.
;;;;   CLOD can then READ an org file and LOAD docstrings into entities.
;;;;   Workflow:
;;;;   Write initial docs in lisp (optional)
;;;;   Run CLOD, produce org file
;;;;   Edit org file
;;;;   Later, after changes in source:
;;;;   Suck org file back into clod, then re-export to org again. The org
;;;;   file will contain updated doc string markers.
;;;;
;;;;
;;;; Generate documentation for this package with:
;;;; (clod:document-package :clod "~/lisp/clod/doc/clod-doc.org"
;;;;     :title "CLOD" :author "Paul Sexton" :email "eeeickythump@gmail.com")
;;;;

(in-package :cl-user)

(declaim (optimize (speed 0) (safety 3) (debug 3)))

(defpackage :clod
  (:use :cl :iterate :closer-mop :cl-ppcre)
  (:shadowing-import-from :closer-mop
                          #:standard-method
                          #:standard-generic-function
                          #:ensure-generic-function
                          #:defmethod
                          #:defgeneric
                          #:standard-class)
  (:export
   #:document-package
   #:document-packages)
  (:documentation
   "* Description

CLOD is a tool for creating documentation for Common Lisp programs.
CLOD examines a loaded package and writes information about all the
symbols defined within that package, to a file.

The output file is in /Org/ format. Org is a simple but powerful 'wiki-like'
markup language that is understood by *Org-mode*, a powerful outliner, personal
wiki and organiser that runs as a major mode within the /Emacs/ text
editor. Org-mode can export org-format files to numerous other formats,
including HTML, LaTeX, PDF, DocBook, and plain text.

More information:
- Emacs :: [[http://www.gnu.org/software/emacs]] (if you program CL, you won't
  need to be told what Emacs is)
- Org mode :: [[http://orgmode.org/]]

* Why use CLOD?

- You can use org markup within docstrings (easy for humans to read) to
  create subsections, bulleted lists, hyperlinks within the document or
  to external URLs, etc.
- Easy export to multiple file formats: text, HTML, DocBook, LaTeX -> PDF...
- You can edit the output within Emacs, which is already the IDE of most
  Commn Lisp programmers.
- If GraphViz is installed, automatically create a diagram illustrating the
  package's class hierarchy -- see http://www.graphviz.org/
- Org markup has many cool tricks. Some examples of useful things you can
  put in docstrings:
  - Include an entire file and format as plain text:
    : #+INCLUDE: \"filename\" quote
  - Include an entire file with common lisp syntax highlighting
    : #+INCLUDE: \"filename.lisp\" src lisp
  - Timestamps and datestamps:
    : {{{ modification-time(%r) }}}
    {{{modification-time(%r)}}}
    : {{{ date(%Y-%m-%d) }}}
    {{{date(%Y-%m-%d)}}}
  - Define text macros, use with ={{{macro(arg1,arg2)}}}=
    : #+MACRO: foo Replacement text $1 $2
  - Embed any LaTeX code directly (no special markup needed)
  - Embed any HTML code (special markup needed)
  - Automatic syntax highlighting of source code examples in exported
    documents, eg:
;;; (defun hello (a &key b)
;;;    (print \"Hello, world!\"))
  - Because CLOD works by introspection rather than parsing files,
    it copes effortlessly with unusual toplevel forms. Documentation
    generators that parse files usually won't be able to tell that your
    toplevel macro `=(defclass-easy ...)=' (or whatever) actually
    expands to a class definition.
  - For the same reason, CLOD avoids the problems with dependencies that
    can trouble other documentation generators (i.e. difficulties
    generating correct documentation unless you manually move things
    around in the source so that the doc generator finds things in
    the `right' order).
  - Easily change the global appearance of the document by specifying a
    cascading style sheet (/note: only affects HTML export from Org mode/)

* Dependencies

- ASDF: [[http://common-lisp.net/project/asdf/]]
- Closer-MOP: [[http://common-lisp.net/project/closer/]]
- Iterate: [[http://common-lisp.net/project/iterate/]]
- CL-PPCRE: [[http://weitz.de/cl-ppcre/]]

* How to use

1. Install CLOD somewhere ASDF can find it.
2. Load CLOD with =(asdf:oos 'asdf:load-op :clod)=
3. Load the package or packages for which you wish to produce documentation,
   eg: =(asdf:oos 'asdf:load-op :mypkg)=
4. Run =(clod:document-package :mypkg nil)= and you will see the documentation
   as a string, returned by [[document-package]].
5. Run =(clod:document-package :mypkg \"filename.org\")= and the same
   documentation will be written to 'filename.org'.
6. Load filename.org into Emacs. Export to HTML with =M-x org-export=, or press
   C-c C-e to be presented with a menu of export options.

* Writing the documentation

All documentation produced by CLOD is obtained by /introspection/, i.e. by the
running CL process examining itself. If a symbol has an associated docstring,
it will be used by CLOD to produce the documentation for that symbol.

Within documentation strings, you can use org mode markup. This is a simple,
human-readable markup language similar to the type of markup used for wiki
editing and forum posting. See the Org
[[http://orgmode.org/manual/Markup.html][manual]] for more information. Also see
the docstrings in the CLOD source code, which use org markup.

Some special points to note:
- Outline headings are used to structure org documents. These headings
  begin with one or more asterisks at the start of the line. Thus, if you
  want a large docstring to be divided into subsections, the heading for
  each subsection should be a line that starts with one or more asterisks (*),
  then a space, then the title of the heading.
- These headings will automatically be correctly 'indented' for their location
  in the structure of the final document. The whole document is one outline,
  and any given docstring will usually be appearing 2 or 3 levels deep within
  that outline. However, CLOD finds all heading lines within docstrings and
  increases the number of asterisks appropriately.
- An extra blank line is also automatically inserted after headings within
  docstrings, allowing you to save screen space in the docstring itself.
- By default, many docstrings are inserted within a subsection titled
  'Description'. However, if you don't want this to happen, but rather want
  the docstring to define its own heading names, make sure that the very first
  thing in the docstring is a heading (straight after the opening quote).
  (Note for mmm-mode users (see below): if the docstring starts with '###'
  to signal that it is in fact a docstring, CLOD will skip the hashes before
  looking to see if the string starts with a heading.)
  So =\"###* Arguments ...\"= will work in that case.
- Some symbol names used by common lisp can conflict with the markup used
  by org mode. For example, =*global-variable*=: asterisks are interpreted
  by org mode as signifying bold text. CLOD catches these in headings and
  auto-generated documentation, but not within doc strings, where you will
  need to surround the offending symbol with =equals signs=.
- *Hyperlinks* are created using
  : [[double square brackets]]
  Any text surrounded by these brackets will link to the same text (case
  insensitive) surrounded by =<<double angle brackets>>=. CLOD uses this to
  define hyperlinks for all symbols in the package. Every symbol MYSYMBOL has:
  1. A hyperlink =<<function MYSYMBOL>>= if MYSYMBOL is a function,
     =<<variable MYSYMBOL>>= if it is a global variable, etc.
  2. A hyperlink =<<MYSYMBOL>>= which will either link to MYSYMBOL's
     documentation, or to a 'disambiguation section' if the same symbol has
     multiple meanings (eg there is both a function and a variable called
     MYSYMBOL).
- Org mode has the ability to use Emacs' font-lock mode to produce source code
  snippets that are correctly syntax highlighted for any major mode.  To use
  this normally requires surrounding the code with =#+BEGIN_SRC ... #+END_SRC=.
  CLOD provides a shortcut: Any lines within docstrings that begin with three
  semicolons =;;;= are assumed to be example lisp source code. The first 3
  semicolons are removed and the rest of the line is syntax highlighted.

* Combining org mode and common lisp mode in a single Emacs buffer

You can use org mode markup within docstrings, but you can't see the effects of
the markup until you export the documentation to org using CLOD. You also don't
get access to org's support for automatic formatting of bulleted lists as you
write, or the fantastic support for writing tables, or hyperlinks that you can
click with the mouse, or ....

What if you could use all the goodness of Org, while editing docstrings in your
lisp source code? You can. This section explains how.

1. Download and install nXhtml, an emacs package that contains code allowing
   multiple major modes to be active in a single buffer.
   http://ourcomments.org/cgi-bin/emacsw32-dl-latest.pl
2. Add the code in `mmm-clod.el' to your .emacs file. Make sure you change
   the mmm-mode directory to the directory where you installed mmm-mode.
3. Restart emacs. Load a lisp source file. All documentation strings should
   appear with a coloured background, and when you move the cursor inside them,
   you will see 'Lisp[Org]' on the modeline.
4. If not everything is highlighting correctly, or if you write a new docstring
   and org does not activate within it, press control-` to 'refresh' mmm-mode.

Not everything works: expanding and collapsing headings fails, and
clicking the mouse elsewhere within the doc string often causes problems. But
overall the two modes work together surprisingly well.

MMM-mode recognises the following things as doc strings:
1. Any string that emacs fontifies using 'font-lock-doc-face'. (in other words,
   font-lock mode must be active.)
2. Any string inside the form '=(:documentation STRING)='.
3. Finally, any string whose first three characters are '###'. Since lines
   beginning with a hash are interpreted as comments by org mode, these
   characters will disappear when you export your document to HTML or other
   formats.

* Example docstring

Here is the docstring for [[document-package]]. It illustrates the use of
headings, bulleted lists, definition lists, =code=, *bold* and /italic/
markup, hyperlinks to other definitions, and syntax highlighting of lisp source
code examples.

: \"* Arguments
: - PKG :: A package name or package object.
: - FILE/STREAM :: A string (filename), stream object, or =NIL=.
: - AUTO-LINKS :: Boolean.
: - LINES-BETWEEN-SECTIONS :: Boolean.
: - BRIEF-METHODS :: Boolean.
: - STYLE-SHEET :: A string.
: - TITLE :: A string.
: - AUTHOR :: A string.
: - EMAIL :: A string.
: * Returns
: A string, or nil.
: * Description
: Produce documentation for the package =PKG=.
:
: The documentation's destination depends on the value of =FILE/STREAM=:
: - =STRING=: documentation is written to the file named by the string.
: - =STREAM=: documentation is written to the already existing stream.
: - =NIL=: documentation is written to a string, which is then returned by
:   this function.
:
: Explanation of optional arguments:
: - =TITLE=, =AUTHOR= and =EMAIL= specify the document title, the name of
:   the author, and the email address of the author.
: - If =AUTO-LINKS= is non-nil, *all* occurrences of symbol names within the
:   text of docstrings will be interpreted as hyperlinks, regardless of
:   whether they are marked up as hyperlinks.
: - If LINES-BETWEEN-SECTIONS is nil, do not output a horizontal line before
:   each new section of documentation.
: - If BRIEF-METHODS is nil, document individual methods with their own
:   sections, just like functions and generic functions. Most people put
:   'method' documentation in the docstrings of their generic functions, but
:   if you set docstrings for individual methods then set this to nil.
: - =STYLE-SHEET= specifies the name of a /Cascading Style Sheet/ (.CSS) file
:   which will be used as the style for the document if you export it
:   to HTML from org mode.\"
:
: * Example
: ;;; (clod:document-package :mypkg \"mypkg-doc.org\"
: ;;;      :style-sheet \"swiss.css\" :title \"My Package\"
: ;;;      :author \"F. B. Quux\" :email \"quux@gmail.com\")
:
: * See also
: - [[document-packages]]
"))

(in-package :clod)

(defparameter *clod-version-string* "1.0"
  "String containing CLOD's version number.")
(defvar *out* *standard-output*
  "Global variable that is bound to the output stream used by CLOD
while writing documentation.")
(defvar *heading-level* 0
  "Number of levels 'deep' within the outline. Used when creating
headings for sections and subsections.")
(defvar *heading-char* #\*
  "Character used at the beginning of lines to signify headings and
subheadings. Should not be changed.")
(defvar *line-width* 80
  "Width to which paragraphs are wrapped, in characters.")
(defvar *left-margin* 0
  "Width of the current 'left margin', in spaces.")
(defvar *ambiguities* nil
  "Hash table created during package documentation. Stores all
symbols which have multiple 'meanings' within the package.")
(defparameter *alphabet*
  (loop for i from 1 to 26 collecting (code-char (+ 64 i)))
  "List of uppercase letters (characters) from A to Z.")
(defvar *hyperspec-root* "http://www.lispworks.com/reference/HyperSpec/"
  "URL or directory where the Hyperspec is found. Not currently
implemented.")
(defvar *auto-links* nil
  "If true, all occurrences of package symbols anywhere in the documentation
will be turned into hyperlinks, even if they are not marked up as such.")
(defvar *lines-between-sections* nil
  "If true, sections of the document will be separated by horizontal lines.")
(defvar *brief-methods* nil
  "If true, most documentation for methods is assumed to be found in the
docstring for their generic function. A generic function's methods are
therefore described in a very brief format (bulleted list).

If false, each method receives its own section, just like other functions.")
(defvar *accessibilities* (list :external :internal)
  "List of one or both of the keywords =:EXTERNAL= and =:INTERNAL=.
Only symbols whose accessibility matches one of the keywords in the list
will be documented.")
(defvar *class-diagram* nil
  "If true, creates a section describing the package class hierarchy as a
'dot' diagram, which can be fed to the GraphViz program (if installed) to
create a visual representation of the hierarchy.")
(defparameter *class-diagram-counter* 0
  "Counter that is incremented by one each time a class diagram is
created. Used to construct the name of the class diagram PNG file that
is produced by `dot'.")
(defvar *document-title* "Documentation"
  "The title of the document. A string.")
(defvar *document-author* "CLOD"
  "The author of the document. A string.")
(defvar *document-email* "your@email.here"
  "The email address of the document's author. A string.")
(defvar *document-style-sheet* nil
  "Filename of the Cascading Style Sheet (.css) file to use if the
document produced by CLOD is exported to HTML.")

(deftype =entity= ()
  "The type 'entity' can have any of several different symbols as its value.
Each value is a different kind of 'meaning' which a symbol can have within
a package. For example, =:function= is a function, =:class= is a class,
and so on."
  `(member :slot :generic-function :function :macro
           :constant :variable :class :goal
           :structure :type :package
           :slot-accessor :slot-writer :slot-reader))


(defgeneric document (sym doctype)
  (:documentation
   "* Arguments
- SYM :: a symbol.
- DOCTYPE :: an [[=entity=]].
* Returns:
Ignored.
* Description
Writes a section documenting the [[=entity=]] named =SYM= which is of entity type
=DOCTYPE=."))


;;;; Utility functions ========================================================


(defmacro do-own-symbols ((var pkg) &body body)
  "* Arguments
- VAR :: symbol naming a variable that will be bound to each symbol in turn.
- PKG :: a package.
* Description
Iterate through all the non-imported symbols in the package =PKG=.
=BODY= is executed once for each such symbol, with =VAR= bound to each
symbol in turn."
  `(let ((%shadow-symbols (package-shadowing-symbols ,pkg)))
     (do-symbols (,var ,pkg)
       (unless (find ,var %shadow-symbols)
         ,@body))))



(defun ampersand-symbol? (sym)
  "Does the symbol SYM begin with an ampersand, such as &ANY, &REST and
so on?"
  (and (symbolp sym)
       (eql #\& (char (format nil "~A" sym) 0))))



(defun entity->tag (entity)
  "* Arguments
- ENTITY :: An [[=entity=]].
* Returns
A string.
* Description
Given an entity, returns a string that can be used as a *tag* denoting that
entity type in org mode. See [[http://orgmode.org/manual/Tags.html]] for
information on tags."
  (case entity
    (:generic-function "generic")
    (:function "function")
    (:macro "macro")
    (:variable "variable")
    (:constant "constant")
    (:class "class")
    (:type "type")
    (:slot "slot")
    (:slot-accessor "reader:writer")
    (:slot-reader "reader")
    (:slot-writer "writer")
    (:structure "structure")
    (:package "package")
    (:goal "goal") 
    (otherwise (error "Unknown entity type: ~S" entity))))


(defun str+ (&rest strings)
  "* Arguments
- STRINGS :: One or more strings.
* Returns
A string.
* Description
Returns the concatenation of all supplied strings. Shorthand
for =(concatenate 'string . STRINGS)=."
  (apply #'concatenate 'string strings))


(defun string-starts-with? (str start)
  "* Arguments
- STR :: A string.
- START :: A smaller string.
* Returns
Boolean.
* Description
Predicate. Does the string =STR= start with the string =START=?
"
  (eql 0 (search start str)))


(defun entity->string (entity)
  "* Arguments
- ENTITY :: An [[=entity=]].
* Returns
A string.
* Description
Given an entity, returns a string that can be used to describe that
entity type in human-readable form, in headings, etc."
  (case entity
    (:generic-function "generic function")
    (:function "function")
    (:macro "macro")
    (:variable "variable")
    (:constant "constant")
    (:class "class")
    (:type "type")
    (:slot "slot")
    (:slot-accessor "slot accessor")
    (:slot-reader "slot reader")
    (:slot-writer "slot writer")
    (:structure "structure")
    (:package "package")
    (:goal "goal")
    (otherwise (error "Unknown entity type: ~S" entity))))


(defun map-list (function list)
  "Map over proper and not proper lists."
  (loop for (car . cdr) on list
        collect (funcall function car) into result
        when (null cdr) return result
        when (atom cdr) return (nconc result (funcall function cdr))))



(defun replace-strings-with-symbols (tree)
  ;; From SLIME. For use with Lispworks arglist introspection.
  (map-list
   (lambda (x)
     (typecase x
       (list
        (replace-strings-with-symbols x))
       (symbol
        x)
       (string
        (intern x))
       (t
        (intern (write-to-string x)))))
   tree))


(defun find-pkg (pkg)
  ;; find-package only seems to work if pkg is an UPPER CASE string
  (find-package (string-upcase (string pkg))))


(defun find-sym (sym &optional (pkg (package-name *package*)))
  ;; find-symbol only seems to work if symbol and pkg are UPPER CASE strings
  (find-symbol (string-upcase (string sym)) (string-upcase (string pkg))))


(defun symbol-accessibility (sym &optional (pkg *package*))
  "* Arguments
- SYM :: A symbol.
- PKG :: A package.
* Returns
One of =:inherited, :internal, :external= or =nil=.
* Description
Returns a symbol describing how the symbol =SYM= is accessible within
the package =PKG=. If =SYM= is exported by =PKG= then the function
returns =:external=, and so on."
  (unless (packagep pkg)
    (setf pkg (find-pkg pkg)))
  (multiple-value-bind (sym2 access) (find-sym (string sym) (package-name pkg))
    (declare (ignore sym2))
    access))


(defun list->string-with-commas (ls)
  "* Arguments
- LS :: A list of values.
* Returns
A string.
* Description
Given a list of arbitrary values, returns a string consisting of the
printed representations of those values, separated by commas and spaces.
* Example
;;; (list->string-with-commas '(a b 123))
;;;
;;; => \"A, B, 123\"
"
  (with-output-to-string (s)
    (if (cdr ls)
        (format s "~{~A, ~}" (butlast ls)))
    (format s "~A" (car (last ls)))))


(defun make-specialised-lambda-list (terms specs)
  "* Arguments
- TERMS :: Unspecialised version of the lambda list (a list of symbols).
- SPECS :: List of class names on which a particular method is
  specialised.
* Returns
A list.
* Description
Given an unspecialised lambda list and a list of specialisers, reconstruct
the specialised lambda list and return it."
  (iterate
    (for term in terms)
    (for spec = (pop specs))
    (if spec
        (collect (list term (make-link spec :class)))
        ;; else
        (collect term))))


;;; The following 2 functions are adapted from SLIME.


(defun declared-special-p (symbol)
  "Returns true if SYMBOL is declared special."
  #+lispworks (sys:declared-special-p symbol)
  #+sbcl (eql :special (sb-int:info :variable :kind symbol))
  #+allegro (eq (sys:variable-information symbol) :special)
  #+clozure (ccl:proclaimed-special-p symbol))



(defun function-lambda-list (func)
  "* Arguments
- FUNC :: A function object, macro object, generic function object,
  or a symbol bound to a function or macro.
* Returns
Two values:
- The lambda list of the function name or function object, FUNC; or nil if
  the function takes no arguments or the lambda list cannot be retrieved.
- A boolean value -- T if a lambda list (even an empty one) was found,
  NIL otherwise.
* Description
Returns the lambda list associated with the definition of the function or
macro =FUNC=. For example, the lambda list for the common lisp function
=FIND= is the list:
: (ITEM SEQUENCE &KEY :FROM-END :TEST :TEST-NOT :START :END :KEY)
"
  (cond
    ((and (not (functionp func))
          (not (fboundp func)))
     (error "Not a function: ~S" func))
    ((and (functionp func)
          (typep func (find-class 'generic-function)))
     (values (generic-function-lambda-list func) t))
    ((and
      (or (symbolp func)                ;fboundp takes either symbol
          (and (consp func)             ;or (setf symbol)
               (cdr func)
               (null (cddr func))
               (eq (first func) 'setf) (symbolp (second func))))
      (fboundp func)
      (typep (symbol-function func) (find-class 'generic-function)))
     (values (generic-function-lambda-list (symbol-function func)) t))
    (t
     #+sbcl
     (let ((llist (sb-introspect:function-lambda-list func)))
       (if llist
           (values llist t)
           (values nil nil)))
     #+allegro
     (handler-case (values (excl:arglist func) t)
       (simple-error () (values nil nil)))
     #+lispworks
     (let ((arglist (lw:function-lambda-list func)))
       (etypecase arglist
         ((member :dont-know)
          (values nil nil))
         (list
          (values (replace-strings-with-symbols arglist) t))))
     #+clozure
     (multiple-value-bind (arglist binding) (let ((*break-on-signals* nil))
                                              (ccl:arglist func))
       (if binding
           (values arglist t)
           (values nil nil)))
     #+armedbear
     (cond ((symbolp func)
            (multiple-value-bind (arglist present)
                (sys::arglist func)
              (when (and (not present)
                         (fboundp func)
                         (typep (symbol-function func)
                                'standard-generic-function))
                (setq arglist
                      (mop::generic-function-lambda-list
                       (symbol-function func))
                      present
                      t))
              (if present
                  (values arglist t)
                  (values nil nil))))
           (t (values nil nil)))
     #+ecl
     (when (or (functionp func) (fboundp func))
       (multiple-value-bind (name fndef)
           (if (functionp func)
               (values (function-name func) func)
               (values func (fdefinition func)))
         (typecase fndef
           (function
            (let ((fle (function-lambda-expression fndef)))
              (case (car fle)
                (si:lambda-block
                 (values (caddr fle) t))
                (t
                 (values nil nil))))))))
     #+cmu
     (let ((llist
             (etypecase func
               (function (cmucl-function-arglist fun))
               (symbol (cmucl-function-arglist (or (macro-function func)
                                                   (symbol-function func)))))))
       (if (eql llist :not-available)
           (values nil nil)
           (values llist t)))
     #+clisp
     (block nil
       (or (ignore-errors
            (return (values (ext:arglist func) t)))
           (ignore-errors
            (let ((exp (function-lambda-expression func)))
              (and exp (return (values (second exp) t)))))
           (values nil nil))))))



(defun function-name (fn)
  "* Arguments
- FN :: A function, generic function or macro object.
* Returns
The name of a function or macro, or nil.
* Description
Returns the official 'name' bound to the function, macro,
or generic function object FN. Returns =NIL= if no name can
be found or if the function is anonymous (=lambda=)."
  (multiple-value-bind (lexp closure-p name)
      (function-lambda-expression fn)
    (declare (ignorable closure-p lexp))
    (or name
        #+allegro
        (nth-value 2 lexp)
        #+lispworks
        (nth-value 2 lexp)
        #+armedbear
        (nth-value 2 lexp)
        #+clozure
        (ccl:function-name fn)
        #+clisp nil
        #+sbcl
        (sb-impl::%fun-name fn)
        #+cmu
        (cond
          ((eval:interpreted-function-p fn)
           (eval:interpreted-function-name fn))
          ((pcl::generic-function-p fn)
           (pcl::generic-function-name fn))
          ((c::byte-function-or-closure-p fn)
           (c::byte-function-name fn))
          (t (kernel:%function-name (kernel:%function-self fn))))
        #+ecl
        (typecase fn
          (generic-function (clos:generic-function-name fn))
          (function (si:compiled-function-name fn)))
        )))




#+cmu
(defun cmucl-function-arglist (fun)
  (let ((arglist
         (cond ((eval:interpreted-function-p fun)
                (eval:interpreted-function-arglist fun))
               ((pcl::generic-function-p fun)
                (pcl:generic-function-lambda-list fun))
               ((c::byte-function-or-closure-p fun)
                (cmucl-byte-code-function-arglist fun))
               ((kernel:%function-arglist (kernel:%function-self fun))
                (handler-case (cmucl-read-arglist fun)
                  (error () :not-available)))
               ;; this should work both for compiled-debug-function
               ;; and for interpreted-debug-function
               (t
                (handler-case (debug-function-arglist
                               (di::function-debug-function fun))
                  (di:unhandled-condition () :not-available))))))
    (check-type arglist (or list (member :not-available)))
    arglist))


#+cmu
(defun cmucl-read-arglist (fn)
  "Parse the arglist-string of the function object FN."
  (let ((string (kernel:%function-arglist
                 (kernel:%function-self fn)))
        (package (find-pkg
                  (c::compiled-debug-info-package
                   (kernel:%code-debug-info
                    (vm::find-code-object fn))))))
    (with-standard-io-syntax
      (let ((*package* (or package *package*)))
        (read-from-string string)))))


#+cmu
(defun cmucl-byte-code-function-arglist (fn)
  ;; There doesn't seem to be much arglist information around for
  ;; byte-code functions.  Use the arg-count and return something like
  ;; (arg0 arg1 ...)
  (etypecase fn
    (c::simple-byte-function
     (loop for i from 0 below (c::simple-byte-function-num-args fn)
           collect (cmucl-make-arg-symbol i)))
    (c::hairy-byte-function
     (cmucl-hairy-byte-function-arglist fn))
    (c::byte-closure
     (cmucl-byte-code-function-arglist (c::byte-closure-function fn)))))


#+cmu
(defun cmucl-make-arg-symbol (i)
  (make-symbol (format nil "~A~D" (string 'arg) i)))


#+cmu
(defun cmucl-hairy-byte-function-arglist (fn)
  (let ((counter -1))
    (flet ((next-arg () (cmucl-make-arg-symbol (incf counter))))
      (with-struct (c::hairy-byte-function- min-args max-args rest-arg-p
                                            keywords-p keywords) fn
        (let ((arglist '())
              (optional (- max-args min-args)))
          (dotimes (i min-args)
            (push (next-arg) arglist))
          (when (plusp optional)
            (push '&optional arglist)
            (dotimes (i optional)
              (push (next-arg) arglist)))
          (when rest-arg-p
            (push '&rest arglist)
            (push (next-arg) arglist))
          (when keywords-p
            (push '&key arglist)
            (loop for (key _ __) in keywords
                  do (push key arglist))
            (when (eq keywords-p :allow-others)
              (push '&allow-other-keys arglist)))
          (nreverse arglist))))))




(defun list-all-direct-slots (classes)
  "* Arguments
- CLASSES :: A list of class objects.
* Return Value
A list of SLOT-DEFINITION instances.
* Description
Return a list of all the direct SLOT-DEFINITION instances defined
for all the classes in CLASSES."
  (let ((slots nil))
    (iterate
      (for c in classes)
      (iterate
        (for slot in (class-direct-slots c))
        (push slot slots)))
    slots))


(defun list-all-indirect-slots (classes)
  "* Arguments
- CLASSES :: A list of class objects.
* Return Value
A list of SLOT-DEFINITION instances.
* Description
Return a list of all the indirect SLOT-DEFINITION instances defined
for all the classes in CLASSES."
  (let ((indirect-slots nil))
    (iterate
      (for c in classes)
      (for direct-slots = nil)
      (iterate
        (for slot in (class-direct-slots c))
        (push slot direct-slots))
      (iterate
        (for slot in (class-slots c))
        (unless (find (slot-definition-name slot) direct-slots
                      :key #'slot-definition-name)
          (push slot indirect-slots))))
    indirect-slots))


(defun list-all-slot-accessors (classes)
  "* Arguments
- CLASSES :: A list of class objects.
* Return Value
A list of generic functions.
* Description
Return a list of all the reader and writer generic functions associated
with all the slots of the classes in CLASSES."
  (let ((slots (remove-if-not
                (lambda (slot) (typep slot (find-class 'direct-slot-definition)))
                (list-all-direct-slots classes))))
    (apply #'append
           (append (mapcar #'slot-definition-readers slots)
                   (mapcar #'slot-definition-writers slots)))))


(defun uses-for-symbol (sym)
  "* Arguments
- SYM :: A symbol.
* Return Value
A list of [[=entity=]] values.
* Description
Given a symbol =SYM=, return a list of entity values, describing the
different meanings/bindings of =SYM= within its home package."
  (let ((uses nil))
    (when (find-pkg sym)
      (push :package uses))
    (cond
      ((find-class sym nil)
       (if (typep (find-class sym) 'structure-class)
           (push :structure uses)
           (push :class uses)))
      ((simple-type? sym)
       (push :type uses)))
    (cond
      ((macro-function sym)
       (push :macro uses))
      ((and (fboundp sym)
            (typep (symbol-function sym) 'COMMON-LISP:standard-generic-function))
       (push :generic-function uses))
      ((fboundp sym)
       (push :function uses)))
    (cond
      ((and (boundp sym)
            (constantp sym))
       (push :constant uses))
      ((boundp sym)
       (push :variable uses)))
    uses))

(defun cram-uses-for-symbol (pkg sym)
  "* Arguments
- SYM :: A symbol.
* Return Value
A list of [[=entity=]] values.
* Description
Given a symbol =SYM=, return a list of entity values, describing the
different cram meanings/bindings of =SYM= within its home package."
  (let ((uses nil)
        (goal nil))
    (setf goal (goal? pkg sym)) 
    (cond 
      (goal
       (progn
         (add-properties-to-found-goal sym goal)
         (push :goal uses))))
    uses))

(defun simple-type? (sym)
  "* Arguments
- SYM :: A symbol.
* Return Value
Boolean.
* Description
Returns =T= if =SYM= names a non-class type, such as can be
defined by [[deftype]]."
  (handler-case (typep t sym)
    (error () (return-from simple-type? nil)))
  t)

(defun goal? (pkg sym)
  (defparameter *pkg* pkg) 
  (let ((goals (get (find-symbol "ACHIEVE" pkg) :goals))
        (found-goal nil))
    (loop for goal in goals
          do 
             (if (eql (car (car (car goal))) sym)
                 (progn
                   (setf found-goal goal))))
    found-goal))

(defun add-properties-to-found-goal (sym goal)
  "* Arguments
* Description
Adds the properties function and arguments of goal to the symbol sym
"
  (setf (symbol-function sym) (second goal))
  (setf (sb-impl::%fun-name (symbol-function sym))sym)
  (setf (get sym 'documentation) (third goal))
  (let ((arguments nil))
    (loop for argument in (car (car goal))
          do 
             ;Skip first entry, its the symbol name. There have to be a way to do it better ! TODO
             (if (not (eql argument (first (car (car goal)))))
                 (progn
                   (setf arguments (append arguments (list argument))))))

    (setf (sb-impl::%fun-lambda-list (symbol-function sym))arguments)
    (setf (get sym 'arguments) arguments)))

(defun word-wrap (text &key (width 80) respect-newlines respect-hyphens
		  exclude-start-char exclude-end-char)
  "* Arguments
- TEXT :: A string.
- WIDTH :: An integer. The maximum length of lines once TEXT is wrapped.
  Default is 80.
- RESPECT-NEWLINES :: Boolean. Should newline characters within the string
  be treated as unbreakable? (=NIL=)
- RESPECT-HYPHENS :: Boolean. Should we refrain from breaking hyphenated
  words? (=NIL=)
- EXCLUDE-START-CHAR :: A character, or nil.
- EXCLUDE-END-CHAR :: A character, or nil.

* Return Value
A list of strings.

* Description
Given a string =TEXT=, breaks the string into a series of
smaller strings, none of which is longer than =WIDTH=. Returns the list of
strings.

If =EXCLUDE-START-CHAR= and =EXCLUDE-END-CHAR= are supplied, those characters
will be treated as demarcating sections of the string whose length is to
be ignored (treated as zero)."
  (iterate
    (with counted = 0)
    (with breakpoint = nil)
    (with skipping = nil)
    (for c :in-string text)
    (for actual :upfrom 0)
    (cond
      ((eql c exclude-start-char)
       (setf skipping t))
      ((eql c exclude-end-char)
       (setf skipping nil)))
    (when (not skipping)
      (incf counted)
      (if (or (eql c #\space) (eql c #\tab)
	      (and (eql c #\Newline) (not respect-newlines))
	      (and (eql c #\-) (not respect-hyphens)))
	  (setf breakpoint actual))
      (when (and (eql c #\Newline) respect-newlines)
	(setf breakpoint actual)
	(setf counted (1+ width))))
    (when (>= counted width)
      (return (cons (substitute-if #\space
				   #'(lambda (ch)
				       (or (eql ch #\tab)
					   (eql ch #\newline)))
				   (subseq text 0
					   (or breakpoint actual)))
		    (word-wrap (subseq text (if breakpoint
						(1+ breakpoint)
						actual))
			       :width width
			       :respect-newlines respect-newlines
			       :respect-hyphens respect-hyphens
			       :exclude-start-char exclude-start-char
			       :exclude-end-char exclude-end-char))))
    (finally (return (list text)))))


;; (eval-when (:compile-toplevel :load-toplevel :execute)
;;   (defparameter *unsafe-symbol-strings*
;;     ;; These SEEM to only be interpreted as formatting directives when they occur
;;     ;; at the beginning of words. Hence the ^ at the beginning of each regex.
;;     '(("^\\*" . "\\*")  ;; bold
;;       ("^/" . "\\/")    ;; italic
;;       ("^\\+" . "\\+")  ;; strikethrough
;;       ("^_" . "\\_")    ;; underline
;;       ("^=" . "\\="))   ;; code
;;     "Alist of =(REGEX . REPLACEMENT)= cons pairs. Used to ensure that symbols
;; do not contain any characters which org will misinterpret as formatting
;; directives.
;;
;; See also: [[org-safe-symbol]]."))


(eval-when (:compile-toplevel :load-toplevel :execute)
  (defun org-safe-symbol (sym)
    "* Arguments
- SYM :: A symbol.
* Return Value
A string.
* Description
Given the symbol SYM, return a string that represents SYM in a form that is
human-readable and where org will not be confused by any characters that
might represent markup instructions."
    (format nil "=~A=" sym)))
    ;; (unless (stringp sym)
    ;;   (setf sym (format nil "~A" sym)))
    ;; (iterate
    ;;   (for (regex . replacement) in *unsafe-symbol-strings*)
    ;;   (setf sym (regex-replace-all regex sym replacement)))
    ;; sym))

(defparameter *unsafe-html-chars*
  '(#\< #\> #\= #\% #\? #\+ #\$ #\/ #\& #\@ #\, #\{ #\} #\^ #\~)
  "List of characters which are unsafe within HTML links and anchors, and
which need to be converted to a safe representation.

See also: [[html-safe-string]].
")


(defun html-safe-string (str)
  "* Arguments
- STR :: A string.
* Returns
A string.
* Description

Given a string which is intended as a link target, return a copy in which we
remove or alter any unsafe characters that will stop the link working properly
when the document is exported to HTML.
* See Also
- [[*unsafe-html-chars*]]
"
  ;; If the link contains ':' then it represents a URL or other special
  ;; link, and is left alone.
  (cond
    ((find #\: str)
     str)
    (t
     (string-downcase
      (apply #'concatenate
             'string
             (iterate
               (for c in-string str)
               (if (find c *unsafe-html-chars*)
                   (collect (format nil "..~2X.." (char-code c)))
                   (collect (make-string 1 :initial-element c)))))))))


;;;; Formatting  ==============================================================


(defun write-preamble ()
  "* Arguments
None.
* Return Value
Ignored.
* Description
Writes some org instructions, intended to be placed at the start of the
document. These specify the document's author, title, and set some
export options."
  (write-out "~&#+TITLE: ~A" *document-title*)
  (write-out "~&#+AUTHOR: ~A" *document-author*)
  (write-out "~&#+EMAIL: ~A" *document-email*)
  (write-out "~&#+LINK: hs ~A/%s" (if (string-starts-with?
                                       *hyperspec-root* "http:")
                                      *hyperspec-root*
                                      (format nil "file:~A" *hyperspec-root*)))
  (if *document-style-sheet*
      (write-out
       "~&#+STYLE: <link rel=\"stylesheet\" type=\"text/css\" href=~S />"
       *document-style-sheet*))
  (write-out "~&#+STARTUP: showall")
  ;; H:NNN = below this many levels deep, headings become bulleted lists
  ;; toc:NNN = go this many levels deep in table of contents
  (write-out "~&#+OPTIONS: toc:4 H:10 @:t tags:nil~%~%"))



(defmacro writing-section ((title) &body body)
  "Wraps all output within the body of the form in its own section."
  `(let* ((*heading-level* (1+ *heading-level*))
          (%title ,title))
     (write-heading (format nil "~:(~A~)" %title))
     ,@body
     (terpri *out*)))



(defmacro writing-section-for-symbol ((entity sym) &body body)
  "Wraps all output within the body of the form in its own section. The
title of the section describes the entity of type =ENTITY= that is
bound to the symbol =SYM=."
  `(let* ((*heading-level* (1+ *heading-level*))
          (%entity ,entity)
          (%sym ,sym)
          (str (format nil "~:(~A~A~): ~(~A~)"
                       (if (symbol-accessibility %sym)
                           (format nil "~A " (symbol-accessibility %sym))
                           "")
                       (entity->string %entity)
                       (org-safe-symbol %sym))))
     ;; Note: targets are situated *before* the heading, otherwise the heading
     ;; is not actually visible when jumping here in HTML
     (if (null (cdr (uses-for-symbol %sym)))   ;; only 1 use - unambiguous
         (format *out* "# link target 2: ~A~%"
                 (make-target %sym)))
     (format *out* "# link target: ~A~%~%" (make-target %sym %entity))
     (write-heading (format nil "~A~V<~A~>" str
                            (- *line-width* (length str))
                            (format nil ":~A:"
                                    (entity->tag %entity))))
     ,@body
     (terpri *out*)))



(defun write-chapter (symlist entity title)
  "* Arguments
- SYMLIST :: A list of symbols.
- ENTITY :: An [[=entity=]].
- TITLE :: A string.
* Return Value
Ignored.
* Description
Writes the section of the document which describes all entities of
type =ENTITY=. =SYMLIST= should be a list of all the symbols that
name such entities."
  (when symlist
    (writing-section (title)
      (dolist (sym (sort symlist #'string< :key #'string))
        (document sym entity)))))



(defun write-docstring (str &key (levels *heading-level*)
                        (default "Not documented."))
  "* Arguments
- STR :: A docstring.
- LEVELS :: How many levels deep in the outline is
  this docstring?
- DEFAULT :: A string. Default text, used if =STR= is =NIL=.
* Return Value
Ignored.
* Description
Writes the docstring STR to the document. Before doing this, processes
the docstring to:
- Demote headings
- Make all hyperlinks safe for org mode and HTML export
- Convert lines beginning with ';;;' to syntax-highlighting markup."
  (cond
    (str
     (with-input-from-string (in str)
       (iterate
         (with src = nil)
         (while (listen in))
         (for line = (read-line in))
         (for str =
              (regex-replace-all
               ;; Regex matches 'heading' lines
               (format nil "^([~A]+)([ \t]+)(.*)$"
                       (quote-meta-chars
                        (format nil "~C" *heading-char*)))
               line
               (lambda (match match1 match2 match3 &rest matches)
                 (declare (ignore match match2 matches))
                 (format nil "~A ~A~%"
                         (make-string (+ (length match1)
                                         (1- levels))
                                      :initial-element *heading-char*)
                         match3))
               :simple-calls t))
         ;; Regex matches first part of "[[...][...]]" long-form
         ;; hyperlinks. Ensures first part of hyperlink is
         ;; html-safe.
         (setf str (regex-replace-all "\\[\\[([^]]*)\\]\\["
                                      str
                                      (lambda (match match1 &rest matches)
                                        (declare (ignore match matches))
                                        (format nil "[[~A]["
                                                (html-safe-string match1)))
                                      :simple-calls t))
         ;; Regex matches all [[...]] short-form hyperlinks.
         ;; Ensures html safety.
         (setf str (regex-replace-all "\\[\\[([^]]*)\\]\\]"
                                      str
                                      (lambda (match match1 &rest matches)
                                        (declare (ignore matches))
                                        (let ((safe (html-safe-string match1)))
                                          (cond
                                            ((string= safe match1)
                                             match)
                                            (t
                                             (format nil "[[~A][~A]]"
                                                     safe match1)))))
                                      :simple-calls t))
         (cond
           ((string-starts-with? str ";;;")
            (unless src
              (setf src t)
              (format *out* "#+BEGIN_SRC lisp~%"))
            (setf str (subseq str 3)))
           (t
            (when src
              (setf src nil)
              (format *out* "#+END_SRC~%"))))
         (format *out* "~A~%" str)
         (finally
          (if src
              (format *out* "#+END_SRC~%"))))))
    (default
     (format *out* "~A~%" default))))


(defmacro write-indented ((indent) &body body)
  "* Arguments
- INDENT :: An integer.
* Return Value
Ignored.
* Description
All text that is written within the body of the form will be indented
a further =INDENT= spaces from the left margin."
  `(let ((*left-margin* (+ *left-margin* ,indent)))
     ,@body))



(defun wrap-and-write (fmt &rest args)
  "Wraps the result of =(FORMAT NIL FMT . ARGS)=, then writes the
resulting paragraph to output."
  (let ((lines (word-wrap (apply #'format nil fmt args) :width *line-width*)))
    (dolist (line lines)
      (write-out line))
    (terpri *out*)))


(defun wrap-and-write-code (fmt &rest args)
  "Wraps the result of =(FORMAT NIL FMT . ARGS)=, then writes the
resulting paragraph with 'literal' or 'source code block' markup."
  (let ((lines (word-wrap (apply #'format nil fmt args)
                          :width (- *line-width* 2))))
    (dolist (line lines)
      (format *out* ": ~A~%" line))
    (terpri *out*)))



(defun write-out (fmt &rest args)
  "Writes the result of =(FORMAT NIL FMT . ARGS)= to output."
  (fresh-line *out*)
  (princ (make-string *left-margin* :initial-element #\space) *out*)
  (apply #'format *out* fmt args))


(defun write-heading (title)
  "Writes a section heading entitled =TITLE=."
  (format *out* "~&~%~A ~A~%~%"
           (make-string *heading-level* :initial-element *heading-char*)
           title))


(defun type->string (typ)
  "Returns a simple string representation of the type specifier =TYP=."
  (format nil "~A"
          (cond
            ((listp typ)
             (car typ))
            (t
             typ))))



(defgeneric make-link (target entity &key text)
  (:documentation
   "* Arguments
- TARGET :: A symbol, string, or other value that constitutes the target of the
  link.
- ENTITY :: An [[=entity=]].
- TEXT :: An optional string, used for the appearance of the link. The default
  is a direct textual representation of =TARGET=.
* Returns
A string.
* Description
Given a target and text, return a string that will be interpreted by org mode as
a hyperlink leading to the documentation for =TARGET=."))

(defmethod make-link ((str string) (entity null) &key text)
  (cond
    (text
     (format nil "~([[~A][~A]]~)" str text))
    (t
     (format nil "~([[~A]]~)" str))))


(defmethod make-link ((str string) (entity symbol) &key text)
  (unless text (setf text str))
  (let ((link (html-safe-string (concatenate 'string (string entity) " " str))))
    (format nil "~([[~A][~A]]~)" link text)))


(defmethod make-link ((sym symbol) (entity t) &key text)
  (make-link (string sym) entity
             :text (or text
                       (format nil "=~S="
                               sym))))


(defmethod make-link ((c class) (entity (eql :class))  &key text)
  (make-link (class-name c) :class :text text))


(defmethod make-link ((o standard-object) (entity (eql :class))  &key text)
  (make-link (class-name (class-of o)) :class :text text))


#-lispworks
(defmethod make-link ((c eql-specializer) (entity (eql :class))  &key text)
  (declare (ignore text))
  (format nil "~S" `(eql ,(eql-specializer-object c))))

#+lispworks
(defmethod make-link (c (entity (eql :class)) &key text)
  (declare (ignorable text))
  (if (clos:eql-specializer-p c)
    (format nil "~S" `(eql ,(eql-specializer-object c)))
    (call-next-method c
                      entity text)))


(defun make-class-link (x &key text)
  "Synonym for ([[make-link]] =X :class=)."
  (make-link x :class :text text))


(defun make-package-link (sym)
  "Synonym for ([[make-link]] =X :package=)."
  (make-link sym :package))


(defun write-list-as-paragraph (ls)
  "LS is a list of items. Writes all the items in LS to output as a
single wrapped paragraph. Individual items are separated by commas."
  (wrap-and-write (list->string-with-commas ls)))




(defun write-docstring-section (title docstr)
  "Writes the documentation string DOCSTR within its own subsection."
  (cond
    ((string-starts-with? docstr (format nil "###~C" #\newline))
      (setf docstr (subseq docstr 4)))
    ((string-starts-with? docstr (format nil "###~C~C" #\return #\newline))
      (setf docstr (subseq docstr 5)))
    ((string-starts-with? docstr (format nil "###"))
      (setf docstr (subseq docstr 3))))
  (if (and (stringp docstr)
           (string-starts-with? docstr (format nil "~C " *heading-char*)))
      (let ((*heading-level* (1+ *heading-level*)))
        (write-docstring docstr :levels *heading-level*))
      ;; else
      (writing-section (title)
        (write-docstring docstr))))


(defun make-target (sym &optional entity)
  "Returns a string that will be interpreted by org as a destination for
hyperlinks to =SYM=."
  (let ((target-start (if *auto-links* "<<<" "<<"))
        (target-end (if *auto-links* ">>>" ">>"))
        (sym (string sym)))
    (if entity
        (format nil "~(~A~A~A~)" target-start
                (html-safe-string (format nil "~A ~A" entity sym))
                target-end)
        (format nil "~(~A~A~A~)" target-start (html-safe-string sym)
                target-end))))


(defmacro writing-bulleted-list (&body body)
  "All output within the body of a form is assumed to be within a
bulleted list."
  `(progn
     ,@body
     (terpri *out*)))


(defun write-bullet-point (fmt &rest args)
  "Writes a the result of =(FORMAT NIL FMT . ARGS)= as a point within
an active bulleted list."
  (let* ((str (apply #'format nil fmt args))
         (lines (word-wrap str :width (- *line-width* 2))))
    (write-out "~&- ~A~%" (car lines))
    (write-indented (2)
      (dolist (line (cdr lines))
        (write-out "~A~%" line)))))



(defun write-index (pkg &optional (accessibilities (list :internal :external)))
  "Writes a section containing an alphabetical index of all the symbols
in the package PKG."
  (let ((symbols nil)
        (index-table (make-hash-table :test 'eql)))
    (do-own-symbols (sym pkg)
      (when (and (or (null accessibilities)
                     (find (symbol-accessibility sym pkg) accessibilities))
                 (uses-for-symbol sym))
        (push sym symbols)))
    (setf symbols (sort symbols #'string>))
    (dolist (sym symbols)
      (push sym (gethash (if (alpha-char-p (elt (string sym) 0))
                             (elt (string sym) 0)
                             'nonalphabetic)
                         index-table)))
    (writing-section ("Index")
      (wrap-and-write
       (format nil
               "~%~{~A  ~}~%"
               (iterate
                 (for ch in (append *alphabet* (list 'nonalphabetic)))
                 (if (gethash ch index-table)
                     (collect (format nil "[[index ~A][~A]]" ch ch))))))
      (iterate
        (for ch in (cons 'nonalphabetic *alphabet*))
        (for syms = (gethash ch index-table))
        (when syms
          (writing-section ((format nil "~A" ch))
            (write-out "~%# link target: <<index ~A>>~%" ch)
            (writing-bulleted-list
              (iterate
                (for sym in syms)
                (iterate
                  (for use in (uses-for-symbol sym))
                  (write-bullet-point
                   "~A, ~:(~A~)"
                   (make-link sym use)
                   (entity->string use)))))))))))



(defun write-disambiguation (sym uses)
  "Writes a section providing disambiguating links for the symbol
SYM, which is bound in multiple namespaces."
  (writing-section (sym)
    (write-out "# target: ~A~%" (make-target (string sym)))
    (write-out "Disambiguation.~%~%")
    (writing-bulleted-list
      (iterate
        (for use in uses)
        (write-bullet-point "~:(~A~): ~A" (entity->string use)
                            (string-downcase
                             (make-link sym use)))))))


(defun write-lambda-list-section (sym)
  "Writes a section describing the lambda list of the function or macro SYM."
  (writing-section ("Syntax")
    (format *out* "~&#+BEGIN_SRC lisp~%")
    (let* ((text (format nil "~A" (cons sym (function-lambda-list sym)))))
      (write-out "~(~A~)~%" text))
    (format *out* "#+END_SRC~%")))



(defun write-colophon ()
  (writing-section ("Colophon")
    (write-out (str+ "This documentation was generated from "
                     "Common Lisp source code using CLOD, version ~A.")
               *clod-version-string*)
    (write-out (str+ "The latest version of CLOD is available "
                     "[[http://bitbucket.org/eeeickythump/clod/][here]]."))))



;;;; Document methods =========================================================



(defmethod document :after ((sym symbol) (doctype t))
  (terpri *out*))


(defmethod document :before ((sym symbol) (doctype t))
  (unless (or (not *lines-between-sections*)
              (eql doctype :method))
    (format *out* "~&-----~%~%")))


(defmethod document ((sym symbol) (doctype (eql :function)))
  (writing-section-for-symbol (:function sym)
    ;; We don't impose any structure (headings) on FUNCTION docstrings,
    ;; because functions are pretty opaque. It is up to the user to
    ;; write everything in the docstring.
    (write-lambda-list-section sym)
    (write-docstring-section "Description" (documentation sym 'function))))

(defmethod document ((sym symbol) (doctype (eql :goal)))
  (print "documenting")
  (print sym)
  (defparameter my-sym sym)
  (writing-section-for-symbol (:goal sym)
    (print "writing lambda")
    (write-lambda-list-section sym)
    (print "lambda") 
    (write-docstring-section "Description" (get sym 'documentation))))

(defmethod document ((sym symbol) (doctype (eql :macro)))
  (writing-section-for-symbol (:macro sym)
    ;; The same goes for macros.
    (write-lambda-list-section sym) 
    (write-docstring-section "Description" (documentation sym 'function))))



(defmethod document ((sym symbol) (doctype (eql :type)))
  (writing-section-for-symbol (:type sym)
    ;; The same goes for macros.
    (let ((*heading-level* (1+ *heading-level*)))
      (write-docstring (documentation sym 'type)))))



(defmethod document ((sym symbol) (doctype (eql :variable)))
  (writing-section-for-symbol (:variable sym)
    (writing-section ("Value")
      (cond
        ((boundp sym)
         (wrap-and-write-code "~S" (symbol-value sym))
         (format *out* "Type: ~(~A~)~%"
                 (org-safe-symbol (type->string (type-of (symbol-value sym))))))
        (t
         (format *out* "Unbound.~%"))))
    (write-docstring-section "Description" (documentation sym 'variable))))


(defmethod document ((sym symbol) (doctype (eql :constant)))
  (writing-section-for-symbol (:constant sym)
    (writing-section ("Value")
      (cond
        ((boundp sym)
         (wrap-and-write-code "~S" (symbol-value sym))
         (format *out* "Type: ~(~A~)~%"
                 (org-safe-symbol (type->string (type-of (symbol-value sym))))))
        (t
         (format *out* "Unbound.~%"))))
    (write-docstring-section "Description" (documentation sym 'variable))))


(defmethod document ((sym symbol) (doctype (eql :generic)))
  (let ((gf (symbol-function sym)))
    (writing-section-for-symbol (:generic-function sym)
      (write-lambda-list-section sym)
      (write-docstring-section "Description" (documentation gf t))
      ;; No portable way for method-combination objects to introspect.
      ;;(format *out* "  Combination: ~S~%"
      ;;        (generic-function-method-combination gf))
      (writing-section ("Methods")
        (dolist (m (generic-function-methods gf))
          (document m :method))))))



(defmethod document ((sym symbol) (doctype (eql :slot-reader)))
  (let ((gf (symbol-function sym)))
    (writing-section-for-symbol (:slot-reader sym)
      (write-lambda-list-section sym)
      (when (documentation gf t)
        (write-docstring-section "Description" (documentation gf t)))
      (writing-section ("Methods")
        (dolist (m (generic-function-methods gf))
          (document m :method))))))



(defmethod document ((sym symbol) (doctype (eql :slot-writer)))
  (let ((gf (symbol-function sym)))
    (writing-section-for-symbol (:slot-writer sym)
      (write-lambda-list-section sym)
      (when (documentation gf t)
        (write-docstring-section "Description" (documentation gf t)))
      (writing-section ("Methods")
        (dolist (m (generic-function-methods gf))
          (document m :method))))))


(defmethod document ((sym symbol) (doctype (eql :slot-accessor)))
  (let ((gf (symbol-function sym)))
    (writing-section-for-symbol (:slot-accessor sym)
      (write-lambda-list-section sym)
      (when (documentation gf t)
        (write-docstring-section "Description" (documentation gf t)))
      (writing-section ("Methods")
        (dolist (m (generic-function-methods gf))
          (document m :method))))))



(defmethod document ((m cl:standard-method) (doctype (eql :method)))
  ;; Methods are just briefly documented, 1-2 lines each.
  ;; Note 'cl:standard-method' above. Avoids problems with
  ;; assuming that all methods are closer-mop:standard-method instances.
  (let* ((method-name (generic-function-name (method-generic-function m)))
         (method-usage
          (format nil
                  "(~(~A~A~{ ~A~}~))"
                  (org-safe-symbol method-name)
                  (if (method-qualifiers m)
                      (format nil "~{ ~S~}" (method-qualifiers m))
                      "")
                  (make-specialised-lambda-list
                   (method-lambda-list m)
                   (method-specializers m)))))
    (cond
      (*brief-methods*
       (write-bullet-point method-usage)
       (if (or (and (slot-exists-p m 'documentation)
                (slot-boundp m 'documentation)
                (documentation m t))
               (documentation m t))
           (write-indented (4)
             (write-docstring (documentation m t)))))
      (t
       (writing-section ("Method")
         (format *out* "~&#+BEGIN_SRC lisp~%")
         (let* ((text (format nil "Method: ~A~A~S" method-usage
                 (if (method-qualifiers m)
                     (format nil "~{ ~S~}"
                             (method-qualifiers m))
                     "")
                 (method-specializers m))))
           (write-out "~(~A~)~%" text))
         (format *out* "#+END_SRC~%")
         (if (or (and (slot-exists-p m 'documentation)
                  (slot-boundp m 'documentation)
                  (documentation m t))
                 (documentation m t))
             (write-docstring-section "Description"
                                      (documentation m t))))))))


;; (defmethod document ((m standard-method) (doctype (eql :method)))
;;   (write-bullet-point
;;    "(~(~A~A~{ ~A~}~))"
;;    (safe-symbol (generic-function-name (method-generic-function m)))
;;    (if (method-qualifiers m)
;;        (format nil "~{ ~S~}" (method-qualifiers m))
;;        "")
;;    (make-specialised-lambda-list
;;     (method-lambda-list m)
;;     (method-specializers m)))
;;   (if (and (slot-exists-p m 'documentation)
;;            (slot-boundp m 'documentation)
;;            (documentation m t))
;;       (write-indented (4)
;;         (write-docstring (documentation m t)))))



(defmethod document ((sym symbol) (doctype (eql :class)))
  (let* ((c (find-class sym)))
    (ensure-finalized c nil)
    (writing-section-for-symbol (:class sym)
      (progn
        (unless (class-finalized-p c)
          (error "Not finalised"))
        (writing-section ("Inheritance")
          (write-bullet-point "Parent classes:")
          (write-indented (4)
            (write-list-as-paragraph
             (or
              (mapcar #'make-class-link
                      (mapcar
                       #'string-downcase
                       (mapcar
                        #'class-name
                        (class-direct-superclasses c))))
              (list "None."))))
          (write-bullet-point "Precedence list:")
          (write-indented (4)
            (write-list-as-paragraph
             (or
              (mapcar #'make-class-link
                      (mapcar
                       #'string-downcase
                       (mapcar
                        #'class-name
                        (class-precedence-list c))))
              (list "None."))))
          (write-bullet-point "Direct subclasses:")
          (write-indented (4)
            (write-list-as-paragraph
             (or
              (mapcar #'make-class-link
                      (mapcar
                       #'string-downcase
                       (mapcar
                        #'class-name
                        (class-direct-subclasses c))))
              (list "None.")))))
        (write-docstring-section "Description" (documentation c t))
        (writing-section ("Direct slots")
          (dolist (slot (class-direct-slots c))
            (document slot :slot)))
        (when (list-all-indirect-slots (list c))
          (writing-section ("Indirect slots")
            (dolist (slot (list-all-indirect-slots (list c)))
              (document slot :slot))))))))



(defmethod document ((sym symbol) (doctype (eql :structure)))
  (let* ((c (find-class sym)))
    (writing-section-for-symbol (:structure sym)
      (progn
        (unless (class-finalized-p c)
          (error "Not finalised"))
        (write-docstring-section "Description" (documentation c t))
        (writing-section ("Slots")
          (dolist (slot (class-direct-slots c))
            (document slot :slot)))))))
;; Structs have the following associated functions: (might NOT be methods)
;;    CONCNAME-SLOTNAME readers (where CONCNAME defaults to STRUCTNAME)
;;    make-STRUCTNAME
;;    STRUCTNAME-p
;;    copy-STRUCTNAME


(defmethod document :around ((slot slot-definition)
                             (doctype (eql :slot)))
  (writing-section-for-symbol (:slot (slot-definition-name slot))
    (writing-bulleted-list
     (write-bullet-point "Value type: ~(~A~)"
                         (org-safe-symbol
                          (type->string (slot-definition-type slot))))
     (write-bullet-point "Initial value: =~S="
                         (slot-definition-initform slot))
     (write-bullet-point "Initargs: ~(~A~)"
                         (if (slot-definition-initargs slot)
                             (list->string-with-commas
                              (slot-definition-initargs slot))
                             'none))
     (write-bullet-point "Allocation: ~(~A~)"
                         (slot-definition-allocation slot)))
    (if (and (slot-exists-p slot 'documentation)
             (slot-boundp slot 'documentation)
             (documentation slot t))
        (write-docstring-section "Description" (documentation slot t)))
    (call-next-method)))


(defmethod document ((slot effective-slot-definition)
                     (doctype (eql :slot)))
  ;; These seem to be 'indirect' slots? They lack a lot of
  ;; information.
  nil)


(defmethod document ((slot direct-slot-definition)
                     (doctype (eql :slot)))
  (let* ((accessors (union (slot-definition-readers slot)
                           (slot-definition-writers slot)))
         (readers (set-difference (slot-definition-readers slot)
                                  accessors :test 'equal))
         (writers (set-difference (slot-definition-writers slot)
                                  accessors :test 'equal)))
    (when (remove-if #'listp accessors)
      (writing-section ("Accessors")
        (dolist (fname (remove-if #'listp accessors))
          (document fname :slot-accessor))))
    (when readers
      (writing-section ("Readers")
        (dolist (fname readers)
          (document fname :slot-reader))))
    (when writers
      (writing-section ("Writers")
        (dolist (fname writers)
          (document fname :slot-writer))))))


;;;; Example classes ==========================================================

;;; We create these purely to illustrate the class diagram
;;; functionality.

(defclass Animal () ()
  (:documentation "dummy class, created to illustrate CLOD's ability
to produce class diagrams using GraphViz."))
(defclass Bird (Animal) ()
  (:documentation "dummy class, created to illustrate CLOD's ability
to produce class diagrams using GraphViz."))
(defclass Mammal (Animal) ()
  (:documentation "dummy class, created to illustrate CLOD's ability
to produce class diagrams using GraphViz."))
(defclass Cat (Mammal) ()
  (:documentation "dummy class, created to illustrate CLOD's ability
to produce class diagrams using GraphViz."))
(defclass Dog (Mammal) ()
  (:documentation "dummy class, created to illustrate CLOD's ability
to produce class diagrams using GraphViz."))
(defclass Hawk (Bird) ()
  (:documentation "dummy class, created to illustrate CLOD's ability
to produce class diagrams using GraphViz."))

;;;; Toplevel =================================================================



(defun document-package-contents (pkg &optional
                                  (accessibilities *accessibilities*))
  "* Arguments
- PKG :: A package name or package object.
- ACCESSIBILITIES :: A list containing zero or more of the symbols
  =:external= or =:internal=.
* Return Value
Ignored.
* Description
Writes documentation for all symbols contained within the package =PKG=.
Does not write documentation for the actual package object."
  (let ((access nil) (uses nil)
        (functions nil)
        (generics nil)
        (macros nil)
        (vars nil)
        (constants nil)
        (structures nil)
        (types nil)
        (classes nil) ;; structures are actually classes too
        (goals nil))  ;; structures are actually classes too

    (unless (packagep pkg)
      (setf pkg (find-pkg pkg)))
    (setf *ambiguities* (make-hash-table :test #'eql))
    (do-own-symbols (sym pkg)
      (setf access (symbol-accessibility sym pkg))
      (when (or (null accessibilities)
                (find access accessibilities))
        (setf uses (uses-for-symbol sym))
        (setf uses (append uses (cram-uses-for-symbol pkg sym)))
        (if (find :class uses) (push sym classes))
        (if (find :structure uses) (push sym structures))
        (if (find :type uses) (push sym types))
        (if (find :macro uses) (push sym macros))
        (if (find :generic-function uses) (push sym generics))
        (if (find :function uses) (push sym functions))
        (if (find :constant uses) (push sym constants))
        (if (find :variable uses) (push sym vars))
        (if (find :goal uses) (push sym goals))
        (if (> (length uses) 1)
            (setf (gethash sym *ambiguities*) (copy-list uses)))))
    
    ;; Remove any generic functions that represent slot accessors
    (setf generics
          (set-difference generics
                          (list-all-slot-accessors
                           (mapcar #'find-class classes))))
    ;; Ensure all classes are finalised
    (dolist (c classes)
      (ensure-finalized (find-class c) nil))
    ;; === main body of definitions ===
    (when (and classes *class-diagram*)
      (writing-section ("Class Hierarchy")
        (write-class-hierarchy (mapcar #'find-class classes))
        ))
    (dolist (access accessibilities)
      (let ((accstr (format nil "~:(~A~) " access)))
        (writing-section ((str+ accstr "Symbols"))
          (write-out "~%~%")
          (write-chapter (accessible-symbols classes access pkg)
                         :class (str+ accstr "Classes"))
          (write-chapter (accessible-symbols structures access pkg)
                         :structure (str+ accstr "Structures"))
          (write-chapter (accessible-symbols types access pkg)
                         :type (str+ accstr "Types"))
          (write-chapter (accessible-symbols constants access pkg)
                         :constant (str+ accstr "Constants"))
          (write-chapter (accessible-symbols vars access pkg)
                         :variable (str+ accstr "Global Variables"))
          (write-chapter (accessible-symbols macros access pkg)
                         :macro (str+ accstr "Macros"))
          (write-chapter (accessible-symbols functions access pkg)
                         :function (str+ accstr "Functions"))
          (write-chapter (accessible-symbols generics access pkg)
                         :generic (str+ accstr "Generic Functions"))
          (write-chapter (cram-accessible-symbols goals access pkg)
                         :goal (str+ accstr "Goals")))))
    (when (plusp (hash-table-count *ambiguities*))
      (writing-section ("Ambiguous Symbols")
        (iterate
          (for (sym uses) in-hashtable *ambiguities*)
          (write-disambiguation sym uses))))
    ;; === index ===
    (write-index pkg accessibilities)))


(defun accessible-symbols (syms access pkg)
  "* Arguments
- SYMS :: A list of symbols.
- ACCESS :: One of the keywords =:INTERNAL= or =:EXTERNAL=.
- PKG :: A package object.
* Returns
A list of symbols.
* Description
Given a list of symbols, SYMS, returns the subset of SYMS whose
accessibility in PKG matches ACCESS."
  (remove-if-not (lambda (sym)
                   (eql (symbol-accessibility sym pkg) access))
                 syms))

(defun cram-accessible-symbols (syms access pkg)
  (defparameter my-syms syms)
  (remove-if-not (lambda (sym)
                   (eql (symbol-accessibility sym pkg) access))
                 syms))

(defun write-lines (&rest lines)
  (dolist (line lines)
    (write-out line)))


(defun write-class-hierarchy (classes)
  "* Arguments
- CLASSES :: A list of CLASS objects.
* Returns
Ignored.
* Description
Writes the inheritance tree of CLASSES as a GraphViz diagram, using the
DOT language. External parent classes are also included.

See http://www.graphviz.org/ for details of the GraphViz language."
  (let ((foreign-parents nil))
    (write-lines
     "# For this to work, you need to have graphviz installed, and the"
     "# program `dot' must be in your PATH."
     "# You also need to enable the optional module 'org-exp-blocks'"
     "# in Emacs' org mode. For this you will need the following line in"
     "# your .emacs file:"
     "#"
     "#   (require 'org-exp-blocks)"
     "#"
     "# If you have trouble getting graphviz to process this diagram,"
     "# try saving the lines between #+begin_dot and #+end_dot to a plain"
     "# text file. Then run the following at the command prompt:"
     "#"
     "#   dot -Tpng file.txt -o class_diagram.png"
     "#"
     "# Then paste a link like"
     "# <img src=\"class_diagram.png\"  alt=\"Class diagram\"/>"
     "# into the html file.")
    (write-out "~&#+begin_dot class-~4,'0d.png -Tpng~%"
               (incf *class-diagram-counter*))
    (write-out "digraph data_relationships {~%")
    (write-out "   rankdir=\"LR\";~%")
    (dolist (c classes)
      (write-out "    \"~:(~A~)\";~%" (class-name c)))
    (dolist (c classes)
      (dolist (parent (class-direct-superclasses c))
        (write-out "   \"~:(~A~)\" -> \"~:(~A~)\";~%"
                   (class-name parent) (class-name c))
        (unless (eql (symbol-package (class-name parent))
                     (symbol-package (class-name c)))
          (push parent foreign-parents))))
    (dolist (fp foreign-parents)
      (write-out "   \"~:(~A~)\" [shape=Mrecord,colour=lightblue,label=\"~:(~A~)|~(~A~)\"];~%"
                 (class-name fp) (class-name fp)
                 (package-name (symbol-package (class-name fp)))))
    (write-out "~&}~%")
    (write-out "~&#+end_dot~%~%")))



(defun docpkg (&rest packages)
  (write-preamble)
  (dolist (pkg (mapcar #'(lambda (p) (if (packagep p) p (find-pkg p)))
                       packages))
    (writing-section-for-symbol (:package (read-from-string (package-name pkg)))
      (writing-bulleted-list
        (write-bullet-point "Uses:")
        (write-indented (4)
          (write-list-as-paragraph
           (mapcar #'make-package-link
                   (mapcar #'string-downcase
                           (mapcar #'package-name (package-use-list pkg))))))
        (write-bullet-point "Used by:")
        (write-indented (4)
          (cond
            ((package-used-by-list pkg)
             (write-list-as-paragraph
              (mapcar #'make-package-link
                      (mapcar #'string-downcase
                              (mapcar #'package-name (package-used-by-list pkg))))))
            (t
             (write-out "None.")))))
      (write-docstring-section "Description" (documentation pkg t))
      (document-package-contents pkg)))
  (write-colophon))



(defun document-packages (packages file/stream
                          &key (auto-links nil)
                          (lines-between-sections t)
                          (brief-methods nil)
                          (internal-symbols? t)
                          (class-diagram nil)
                          (style-sheet nil)
                          (title *document-title*)
                          (author *document-author*)
                          (email *document-email*))
  "* Arguments
- PACKAGES :: A list of package objects, or symbols naming packages.
- FILE/STREAM :: A string (filename), stream object, or =NIL=.
Other arguments are the same as for [[document-package]].
* Returns
A string, or nil.
* Description
Produces documentation for all the packages in =PACKAGES=, within a
single file.

See [[document-package]] for more details."
  (let ((*auto-links* auto-links)
        (*lines-between-sections* lines-between-sections)
        (*brief-methods* brief-methods)
        (*accessibilities* (if internal-symbols?
                               (list :external :internal)
                               (list :external)))
        (*class-diagram* class-diagram)
        (*document-style-sheet* style-sheet)
        (*document-title* title)
        (*document-author* author)
        (*document-email* email))
    (cond
      ((streamp file/stream)
       (let ((*out* file/stream))
         (apply #'docpkg packages)))
      ((stringp file/stream)
       (with-open-file (*out* file/stream :direction :output
                              :if-exists :supersede)
         (apply #'docpkg packages)))
      ((null file/stream)
       (with-output-to-string (*out*)
         (apply #'docpkg packages))))))


(defun document-package (pkg file/stream &key (auto-links nil)
                          (lines-between-sections t)
                          (brief-methods nil)
                         (internal-symbols? t)
                         (class-diagram nil)
                         (style-sheet nil)
                         (title nil) (author *document-author*)
                         (email *document-email*))
  "* Arguments
- PKG :: A package name or package object.
- FILE/STREAM :: A string (filename), stream object, or =NIL=.
- AUTO-LINKS :: Boolean.
- LINES-BETWEEN-SECTIONS :: Boolean.
- BRIEF-METHODS :: Boolean.
- INTERNAL-SYMBOLS :: Boolean.
- CLASS-DIAGRAM :: Boolean.
- STYLE-SHEET :: A string.
- TITLE :: A string.
- AUTHOR :: A string.
- EMAIL :: A string.
* Returns
A string, or nil.
* Description
Produce documentation for the package =PKG=.

The documentation's destination depends on the value of =FILE/STREAM=:
- =STRING=: documentation is written to the file named by the string.
- =STREAM=: documentation is written to the already existing stream.
- =NIL=: documentation is written to a string, which is then returned by
  this function.

Explanation of optional arguments:
- =TITLE=, =AUTHOR= and =EMAIL= specify the document title, the name of
  the author, and the email address of the author.
- =STYLE-SHEET= specifies the name of a Cascading Style Sheet (.CSS) file
  which will be used as the style for the document if you export it
  to HTML from org mode.
- If =AUTO-LINKS= is non-nil, *all* occurrences of symbol names within the
  text of docstrings will be interpreted as hyperlinks, regardless of
  whether they are marked up as hyperlinks.
- If =LINES-BETWEEN-SECTIONS= is nil, do not output a horizontal line before
  each new section of documentation.
- If =BRIEF-METHODS= is nil, document individual methods with their own
  sections, just like functions and generic functions. Most people put
  'method' documentation in the docstrings of their generic functions, but
  if you set docstrings for individual methods then set this to nil.
- If =INTERNAL-SYMBOLS?= is non-nil, document both internal and external
  (exported) symbols. If nil, only document external symbols.
- If =CLASS-DIAGRAM= is non-nil, create a section after the toplevel package
  description, containing a description of the package hierarchy
  in the form of a GraphViz 'dot' diagram (see http://www.graphviz.org/).

* Example
;;; (clod:document-package :mypkg \"mypkg-doc.org\"
;;;      :style-sheet \"swiss.css\" :title \"My Package\"
;;;      :author \"F. B. Quux\" :email \"quux@gmail.com\")

* See also
- [[document-packages]]"
  (unless (packagep pkg)
    (setf pkg (find-pkg pkg)))
  (unless title
    (setf title (format nil "The ~A package"
                        (package-name pkg))))
  (document-packages (list pkg) file/stream
                     :auto-links auto-links
                     :lines-between-sections lines-between-sections
                     :brief-methods brief-methods
                     :internal-symbols? internal-symbols?
                     :class-diagram class-diagram
                     :style-sheet style-sheet
                     :title title :author author :email email))
