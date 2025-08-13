
(cl:in-package :asdf)

(defsystem "tiago_move-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :moveit_msgs-msg
)
  :components ((:file "_package")
    (:file "GenerateGrasps" :depends-on ("_package_GenerateGrasps"))
    (:file "_package_GenerateGrasps" :depends-on ("_package"))
  ))