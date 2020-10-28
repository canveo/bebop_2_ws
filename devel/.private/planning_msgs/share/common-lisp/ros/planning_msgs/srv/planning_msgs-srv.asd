
(cl:in-package :asdf)

(defsystem "planning_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :planning_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "PlannerService" :depends-on ("_package_PlannerService"))
    (:file "_package_PlannerService" :depends-on ("_package"))
  ))