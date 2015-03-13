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


(asdf:defsystem clod
  :name "clod"
  :version "1.0.0"
  :author "Paul Sexton"
  :description "Common Lisp Autodoc generator"
  :serial t     ; Only true for the simple case         .
  :depends-on ("iterate" "closer-mop" "cl-ppcre"
                         #+sbcl "sb-introspect")
  :components
  ((:file "clod")))
