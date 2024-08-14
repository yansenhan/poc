
(cl:in-package :asdf)

(defsystem "ucar_nav-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Boxinfo" :depends-on ("_package_Boxinfo"))
    (:file "_package_Boxinfo" :depends-on ("_package"))
  ))