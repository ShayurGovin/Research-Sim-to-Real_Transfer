
(cl:in-package :asdf)

(defsystem "naoqi_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AudioCustomMsg" :depends-on ("_package_AudioCustomMsg"))
    (:file "_package_AudioCustomMsg" :depends-on ("_package"))
  ))