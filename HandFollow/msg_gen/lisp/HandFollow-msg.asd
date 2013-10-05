
(cl:in-package :asdf)

(defsystem "HandFollow-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PidParams" :depends-on ("_package_PidParams"))
    (:file "_package_PidParams" :depends-on ("_package"))
    (:file "vw" :depends-on ("_package_vw"))
    (:file "_package_vw" :depends-on ("_package"))
  ))