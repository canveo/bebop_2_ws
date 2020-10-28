#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int16
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged as AttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged as AltitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged as SpeedChaged
from nav_msgs.msg import Odometry

# Ardrone3PilotingStateFlyingStateChanged  subscriber estado de vuelo

from math import sin, cos
# import tf
# ass = tf.transformations.quaternion_from_euler()

# import threading
# import time
# from time import strftim
# from pid import PID
import numpy as np

import collections
import csv

from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix
from PID import PID

class Trayectoria:   
    
    def __init__(self):

        self.velX = 0
        self.velY = 0
        self.altura = 0
        self.yaw = 0

        self.quatX = 0
        self.quatY = 0
        self.quatZ = 0
        self.quatW = 0

        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.posW = 0
         
        # inicio valores de odometria
        self.transX = 0
        self.transY = 0
        self.transZ = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0
        
        self.timeSecPrev = rospy.Time.now()

        # rospy.Subscriber("velX", Float32, self.callback0)
        # rospy.Subscriber("velY", Float32, self.callback1)
        # rospy.Subscriber("altura", Float32, self.callback2)
        # rospy.Subscriber("timeSec", Float32, callback4)
        rospy.Subscriber("bebop/states/ardrone3/PilotingState/AttitudeChanged", AttitudeChanged, self.callback3) # YAW
        rospy.Subscriber("bebop/states/ardrone3/PilotingState/AltitudeChanged", AltitudeChanged, self.callback4) # altura bebop

        rospy.Subscriber("bebop/odom", Odometry, self.callback5) #  posicion y orientacion
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %f %f %f", self.velX, self.velY, self.altura)  

        self._x = 0
        self._y = 0 

        # Publisher pid output
        self.pub0 = rospy.Publisher('pid_altura', Float32, queue_size=10) 
        self.pub1 = rospy.Publisher('pid_velX', Float32, queue_size=10)
        self.pub2 = rospy.Publisher('pid_velY', Float32, queue_size=10)

        # Controlador de altura

        #----- Modifiable Parameters
        # PID controller
        # Proportional parameters XYZrpy
        kp = np.array([0.15, 0.15, 0.2, 0, 0, 0.15])
        # Integral parameters XYZrpy
        ki = np.array([0.01, 0.01, 0.01, 0, 0, 0.01])
        # Proportional parameters XYZrpy
        kd = np.array([0.25, 0.25, 0.01, 0, 0, 0.25])

        samplingTime = 0.1  # 100 [ms]

        self.waypoints = [np.array([0.0, 0.0, 1.1, 0.0, 0.0, np.radians(30)]),
                          np.array([0.0, 0.0, 1.1, 0.0, 0.0, np.radians(30)]),
                          np.array([0.0, 0.0, 1.1, 0.0, 0.0, np.radians(45)]),
                          np.array([0.0, 0.0, 1.1, 0.0, 0.0, np.radians(60)])]

        self.freqTopic = 10  # Frequency of topic messages

        self.stableTime = 5  # Time in the vecinity of target waypoint 5[s]
        self.inReach = 0.1    # Distance to be considered in the vecinity of the waypoint

        self.fileName = "hw04_task01_pid.csv"    # Name of the file to save the data

        # ----- Initialization stage
        self.controller = PID()  # controller
        self.controller.setKp(kp)
        self.controller.setKi(ki)
        self.controller.setKd(kd)

        np.set_printoptions(precision=3) # to print numpy

        self.controller.setSamplingTime(samplingTime)

        self.waypointIndex = 0      # Index to track current waypoints
        self.controller.setSetPoint(self.waypoints[self.waypointIndex])

        # List to have a mean of messages to make sure the distance do not oscillate much
        self.thresholdList = collections.deque(
            maxlen=(self.stableTime*self.freqTopic))

        # Initialize the CSV
        #
        with open(self.fileName, mode='w') as data_file:
                    file_writter = csv.writer(
                        data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    file_writter.writerow([0, 0, 0, 0, 0])

        # # Initialize the node
        # rospy.init_node('drone_controller', anonymous=True)
        # self.rate = rospy.Rate(self.freqTopic)  # 10hz

    
        # Parametros del check



    def callback0(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " velX %f", data.data)
        self.velX = round(data.data, 2)

    def callback1(self,data):
        # rospy.loginfo(rospy.get_caller_id() + " velY %f", data.data)
        self.velY = round(data.data, 2)

    def callback2(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " altura %f", data.data)
        self.altura = round(data.data, 2)

    def callback3(self, data):
        # YAW Bebop
        # rospy.loginfo(rospy.get_caller_id() + " yaw %f\n\f\r", data.yaw)
        self.yaw = data.yaw
    
    def callback4(self, data):
        # Altitud Bebop
        self.altura_b = data.altitude

    def callback5(self, data):
        self.quatX = data.pose.pose.orientation.x
        self.quatY = data.pose.pose.orientation.y
        self.quatZ = data.pose.pose.orientation.z
        self.quatW = data.pose.pose.orientation.w        
                
        self.posX = data.pose.pose.position.x
        self.posY = data.pose.pose.position.y
        self.posZ = data.pose.pose.position.z

    def checkGoal(self, pose):
        # check si se alcanza el objetivo
        dronePosInertial = self.odometryMsg2InertialCoordinates()

        distance = math.sqrt((dronePosInertial[0]-self.controller.getSetPoint()[0])**2
                             + (dronePosInertial[1]-self.controller.getSetPoint()[1])**2
                             + (dronePosInertial[2]-self.controller.getSetPoint()[2])**2)

        self.thresholdList.append(distance) # Save into the list
        

        if self.thresholdList > 0:
            average = sum(self.thresholdList) / len(self.thresholdList)

        if (average < self.inReach):
            self.waypointIndex = self.waypointIndex + 1 
            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
            self.currentGoal = self.waypoints[self.waypointIndex]

        # print("X:{x} Y:{y} Z: {r} PSI:{p}".format(
            # x=dronePosInertial[0], y=dronePosInertial[1], r=dronePosInertial[2], p=dronePosInertial[3]))
        # print("Goal: {g} Average Distance: {a} Distances averaged: {l}".format(g=self.currentGoal, a=average, l=len(self.thresholdList)))

	
    
    def calcTrajectoria(self):
        # self.timeSecPrev = rospy.Time.now()
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(8)

        while True:
            
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()

            self._x += self.velX * dt
            self._y += self.velY * dt
            print(self._x, self._y, self.altura)

            self.odometryMsg2InertialCoordinates()
            # self.droneController(odom)
            # self.checkGoal()
            # print(self.dronePosInertialWAng)

            self.droneController()
            

            # publisher
            # self.pub0.publish(alt_pid)
            # self.pub1.publish(posX_pid)
            # self.pub2.publish(posY_pid) 
            
            last_time = current_time
            r.sleep()
            

    def odometryMsg2InertialCoordinates(self):
        """Convert an Odometry message to Inertial Coordinates (x,y,z,psi)"""
        angles = euler_from_quaternion((self.quatX, self.quatY, self.quatZ, self.quatW))
        
        self.phi = angles[0]
        self.theta = angles[1]
        self.psi = angles[2]
        # X, Y, Z
        dronePos = np.array([self.posX, self.posY, self.posZ])
 
        # Inertia into body coordinates transformation matrix
        cp = np.cos(self.phi)
        sp = np.sin(self.phi)
        ct = np.cos(self.theta)
        st = np.sin(self.theta)
        cs = np.cos(self.psi)
        ss = np.sin(self.psi)

        transMat = np.array([[ct*cs,          ct*ss,          -st],
                             [sp*st*cs-cp*ss, sp*st*ss+cp*cs, sp*ct],
                             [cp*st*cs+sp*ss, cp*st*ss-sp*cs, cp*ct]])

        dronePosInertial = transMat.dot(np.transpose(dronePos))     # Drone coordinates in body coordinates

        self.dronePosInertialWAng = np.concatenate([dronePosInertial,np.array([self.phi,self.theta,self.psi])])

        # return dronePosInertialWAng

    def droneController(self):
        """"Control the drone"""

        # dronePosInertial = self.odometryMsg2InertialCoordinates()
        # print(dronePosInertial)
        self.ctrlOutput = self.controller.controll(self.dronePosInertialWAng, rospy.Time.now().to_time())

        # Create command
        # print(self.ctrlOutput)
        # if ctrlOutput is not None :
        #     self.cmdMsg.linear.x = ctrlOutput[0]
        #     self.cmdMsg.linear.y = ctrlOutput[1]
        #     self.cmdMsg.linear.z = ctrlOutput[2]
        #     self.cmdMsg.angular.z = ctrlOutput[5]

        #     # Publish
        #     self.cmdPub.publish(self.cmdMsg) 
           

if __name__ == "__main__":
    rospy.init_node('pid_xyz', anonymous=True)
    tr = Trayectoria()
    tr.calcTrajectoria()
    # plt.ion()
    # plt.show()
    rospy.spin()
    
    
    

    
