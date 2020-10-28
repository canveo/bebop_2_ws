#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
import numpy as np
import tf

class SPbebop:

    def __init__(self):

        self.waypoints = [np.array([0, 0, 2, 0, 0, -1.54]),
                          np.array([0, 0, 2, 0, 0, -1.54]),
                          np.array([1, 0, 2, 0, 0, -1.54]),
                          np.array([2, 0, 2, 0, 0, -1.54]),
                          np.array([3, 0, 2, 0, 0, -1.54]),
                          np.array([4, 0, 2, 0, 0, -1.54]),
                          np.array([4, 1, 2, 0, 0, -1.54]),
                          np.array([4, 2, 2, 0, 0, -1.54]),
                          np.array([4, 3, 2, 0, 0, -1.54]),
                          np.array([4, 4, 2, 0, 0, -1.54]),
                          np.array([3, 4, 2, 0, 0, -1.54]),
                          np.array([2, 4, 2, 0, 0, -1.54]),
                          np.array([1, 4, 2, 0, 0, -1.54]),
                          np.array([0, 4, 2, 0, 0, -1.54]),
                          np.array([0, 3, 2, 0, 0, -1.54]),
                          np.array([0, 2, 2, 0, 0, -1.54]),
                          np.array([0, 1, 2, 0, 0, -1.54]),
                          np.array([0, 0, 2, 0, 0, -1.54])]

        rospy.init_node('bebop_setpoint', anonymous=True)

        self.pubSPX = rospy.Publisher('setpoint_X', Float64, queue_size=10)
        self.pubSPY = rospy.Publisher('setpoint_Y', Float64, queue_size=10)
        self.pubSPZ = rospy.Publisher('setpoint_Z', Float64, queue_size=10)
        self.pubSPYaw = rospy.Publisher('setpoint_Yaw', Float64, queue_size=10)
       
       

    def pubsetpoint(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.pubSPX.publish(0.0)
            self.pubSPY.publish(0.0)
            self.pubSPZ.publish(2.0)
            self.pubSPYaw.publish(-1.54)
            rate.sleep()

if __name__ == "__main__":
    try:
        odom = SPbebop()
        odom.pubsetpoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

