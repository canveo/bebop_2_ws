#!/usr/bin/env python
from px4flow import PX4Flow
import rospy
from std_msgs.msg import Float32, Int16


class ShowFlow(PX4Flow):

    rospy.init_node('flujoOptico', anonymous=True)
    pub0 = rospy.Publisher('altura', Float32, queue_size=10) 
    pub1 = rospy.Publisher('velX', Float32, queue_size=10)
    pub2 = rospy.Publisher('velY', Float32, queue_size=10)
    # pub3 = rospy.Publisher('timeSec', Int16, queue_size=10)   
    rate = rospy.Rate(10) # 10hz   

    def update(self):

        alt = self.getGroundDistance()
        _x, _y = self.getFlowComp()
        _timeSec = self.getTime
        # while not rospy.is_shutdown():          
        rospy.loginfo(alt)
        self.pub0.publish(alt)
        rospy.loginfo(_x)
        self.pub1.publish(_x)
        rospy.loginfo(_y)
        self.pub2.publish(_y)
        # rospy.loginfo(_timeSec)
        # self.pub2.publish(_timeSec)
        self.rate.sleep()
        

if __name__ == "__main__":
    
    sensor = ShowFlow('/dev/ttyACM0')
    
    while True:
        sensor.refresh()
