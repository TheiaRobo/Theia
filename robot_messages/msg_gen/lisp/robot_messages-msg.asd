
(cl:in-package :asdf)

(defsystem "robot_messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "coords" :depends-on ("_package_coords"))
    (:file "_package_coords" :depends-on ("_package"))
  ))