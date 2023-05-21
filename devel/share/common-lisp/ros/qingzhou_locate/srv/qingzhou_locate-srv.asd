
(cl:in-package :asdf)

(defsystem "qingzhou_locate-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RobotLocation" :depends-on ("_package_RobotLocation"))
    (:file "_package_RobotLocation" :depends-on ("_package"))
  ))