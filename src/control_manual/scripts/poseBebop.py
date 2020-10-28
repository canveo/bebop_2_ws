#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
import numpy as np
import tf

class poseBebop:

    def __init__(self):

        rospy.init_node('bebopPose', anonymous=True)

        self.pubPoseX = rospy.Publisher('/pos_X/state', Float64, queue_size=10)
        self.pubPoseY = rospy.Publisher('/pos_Y/state', Float64, queue_size=10)
        self.pubPoseZ = rospy.Publisher('/pos_Z/state', Float64, queue_size=10)
        self.pubPoseYaw = rospy.Publisher('/pos_Yaw/state', Float64, queue_size=10)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            poseSubscriber = rospy.Subscriber('bebop/odom', Odometry , self.callbackOdometry)
            rate.sleep()


    def callbackOdometry(self, message):
        pos = message.pose.pose
        quat = pos.orientation

        # Transform quaternion coordinates to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                            quat.z, quat.w))

        phi = angles[0]
        theta = angles[1]
        psi = angles[2]

        # X, Y, Z
        dronePos = np.array([pos.position.x, pos.position.y, pos.position.z])

        # Inertia into body coordinates transformation matrix
        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        cs = np.cos(psi)
        ss = np.sin(psi)

        transMat = np.array([[ct*cs,          ct*ss,          -st],
                                [sp*st*cs-cp*ss, sp*st*ss+cp*cs, sp*ct],
                                [cp*st*cs+sp*ss, cp*st*ss-sp*cs, cp*ct]])

        dronePosInertial = transMat.dot(np.transpose(dronePos))     # Drone coordinates in body coordinates

        dronePosInertialWAng = np.concatenate([dronePosInertial,np.array([phi,theta,psi])])
        self._x = dronePosInertialWAng[0]
        self._y = dronePosInertialWAng[1]
        self._z = dronePosInertialWAng[2]
        self._yaw = dronePosInertialWAng[5]
   
        self.pubPoseX.publish(pos.position.x)
        self.pubPoseY.publish(pos.position.y)
        self.pubPoseZ.publish(pos.position.z)
        self.pubPoseYaw.publish(self._yaw)

        # return dronePosInertialWAng

    # def pubOdom(self):
    #     print(self._z)
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         self.pubPoseX.publish(self._x)
    #         self.pubPoseY.publish(self._y)
    #         self.pubPoseZ.publish(self._z)
    #         self.pubPoseYaw.publish(self._yaw)
    #         rate.sleep()
    #         print("hola")
              

if __name__ == "__main__":
    try:
        odom = poseBebop()
        # odom.pubOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

