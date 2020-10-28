
(cl:in-package :asdf)

(defsystem "planning_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PointCloudWithPose" :depends-on ("_package_PointCloudWithPose"))
    (:file "_package_PointCloudWithPose" :depends-on ("_package"))
    (:file "PolynomialSegment4D" :depends-on ("_package_PolynomialSegment4D"))
    (:file "_package_PolynomialSegment4D" :depends-on ("_package"))
    (:file "PolynomialTrajectory4D" :depends-on ("_package_PolynomialTrajectory4D"))
    (:file "_package_PolynomialTrajectory4D" :depends-on ("_package"))
  ))