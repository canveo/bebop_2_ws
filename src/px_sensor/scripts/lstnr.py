#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int16
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged as AttitudeChanged

from math import sin, cos
# import tf
# ass = tf.transformations.quaternion_from_euler()

import threading
import time
from time import strftime


global velX, velY, altura, yaw, timeSecPrev 
velX = 0
velY = 0
altura = 0
timeSecPrev = None

# filename = 'bebop_datalog/px4flow_prueba_' + strftime('%d_%b_%Y_%H_%M_%S') + '.csv'
# logfile = open(filename, 'wb')

def callback0(data):
    global velX
    rospy.loginfo(rospy.get_caller_id() + " velX %f", data.data)
    velX = data.data

def callback1(data):
    global velY
    rospy.loginfo(rospy.get_caller_id() + " velY %f", data.data)
    velY = data.data

def callback2(data):
    global altura
    rospy.loginfo(rospy.get_caller_id() + " altura %f", data.data)
    altura = data.data

def callback3(data):
    global yaw
    rospy.loginfo(rospy.get_caller_id() + " yaw %f\n\f\r", data.yaw)
    yaw = data.yaw

def callback4(data):
    global timeSec
    # rospy.loginfo(rospy.get_caller_id() + " timeSec %f\n\f\r", data.data)
    # timeSec = data.data
    

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("velX", Float32, callback0)
    rospy.Subscriber("velY", Float32, callback1)
    rospy.Subscriber("altura", Float32, callback2)
    # rospy.Subscriber("timeSec", Float32, callback4)
    rospy.Subscriber("bebop/states/ardrone3/PilotingState/AttitudeChanged", AttitudeChanged, callback3)
    rospy.loginfo(rospy.get_caller_id() + "I heard %f %f %f", velX, velY, altura)   
    
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped


def trajectoria(velX, velY, altura, yaw, timeSecPrev):
    # global velX, velY, altura, yaw, timeSecPrev 
    while True:
        timeSec = time.time()
        print(time.time())
        if timeSecPrev:
            dt = timeSec - timeSecPrev
            # print(dt)
            if dt < 0.3:
                # print("otro aqui")
                pos_x = (velY * cos(yaw) - velX * sin(yaw)) * dt
                pos_y = (velX * cos(yaw) + velY * sin(yaw)) * dt
                logfile.write('%3.3f, %3.3f, %3.3f' % (pos_x, pos_y, yaw))
                logfile.write('\n')
                logfile.flush()
            timeSecPrev = timeSec 
        timeSecPrev = timeSec
        # print(timeSecPrev)

if __name__ == '__main__':
    
    listener()

    # t = threading.Thread(target=posicion)
    # t.start()
    # t.join()
        