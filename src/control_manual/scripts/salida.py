#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Twist, Vector3


class Salida:
    def __init__(self):

        rospy.init_node('bebopOutput', anonymous=True)
        self.pid_X = rospy.Subscriber("/pos_X/control_effort", Float64, self.subpidX)
        self.pid_Y = rospy.Subscriber("/pos_Y/control_effort", Float64, self.subpidY)
        self.pid_Z = rospy.Subscriber("/pos_Z/control_effort", Float64, self.subpidZ)
        self.pid_Yaw = rospy.Subscriber("/pos_Yaw/control_effort", Float64, self.subpidYAW)
        self.pubcmd = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)

    def subpidX(self, data):
        self.pid_X = data.data

    def subpidY(self, data):
        self.pid_Y = data.data


    def subpidZ(self, data):
        self.pid_Z = data.data
        # print(self.pid_Z)
    
    def subpidYAW(self, data):
        self.pid_Yaw = data.data
       

    def mostrar(self):
        # print(self.pid_Z)
        while not rospy.is_shutdown():

            self.twist = Twist()
            self.twist.linear.x = self.pid_X
            self.twist.linear.y = self.pid_Y
            self.twist.linear.z = self.pid_Z
            self.twist.angular.z = self.pid_Yaw

            self.pubcmd.publish(self.twist)
         

if __name__ == "__main__":
    try:
        out = Salida()
        out.mostrar()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass