#!/usr/bin/env python

import rospy
# from tum_ardrone.msg import filter_state
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import Empty
# from PIDAutoTuning import PIDAutoTuning
from nav_msgs.msg import Odometry   # linea nueva

from PID import PID
import numpy as np
import math
import collections
import csv

class Controller:

    def __init__(self):
        
        #----- Modifiable Parameters
        # PID controller
        # Proportional parameters XYZrpy
        kp = np.array([0.15, 0.15, 0.2, 0, 0, 0.15])
        # Integral parameters XYZrpy
        ki = np.array([0.01, 0.01, 0.01, 0, 0, 0.01])
        # Proportional parameters XYZrpy
        kd = np.array([0.25, 0.25, 0.01, 0, 0, 0.25])

        samplingTime = 0.1  # 100 [ms]

        self.waypoints = [np.array([0, 0, 2, 0, 0, 0]),
                          np.array([0, 6, 2, 0, 0, 0]),
                          np.array([6, 6, 2, 0, 0, 0]),
                          np.array([6, 0, 2, 0, 0, 0])]
        self.freqTopic = 10  # Frequency of topic messages

        self.stableTime = 5  # Time in the vecinity of target waypoint 5[s]
        self.inReach = 0.1    # Distance to be considered in the vecinity of the waypoint

        self.fileName = "hw04_task01.csv"    # Name of the file to save the data

        # ----- Initialization stage
        self.controller = PID()  # controller
        self.controller.setKp(kp)
        self.controller.setKi(ki)
        self.controller.setKd(kd)

    
        self.controller.setSamplingTime(samplingTime)



        self.now=rospy.get_rostime().to_sec()

        self.pubTakeoff = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.pubLand = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.pubcmd = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.pub_pitch = rospy.Publisher("/bebop/pitch_pid", Vector3, queue_size=10)
        self.pub_roll = rospy.Publisher("/bebop/roll_pid", Vector3, queue_size=10)
        self.pub_alt = rospy.Publisher("/bebop/alt_pid", Vector3, queue_size=10)
        self.pub_error = rospy.Publisher("/bebop/error", Vector3, queue_size=10)
        self.pub_target = rospy.Publisher("/bebop/target", Vector3, queue_size=10)

        self.target_count = 0

        rospy.Subscriber("/bebop/odom", Odometry, self.odometry) 

        self.posX = 0.0
        self.posY = 0.0
        self.posZ = 0.0
    
    def odometry(self, data):
        posX = data.pose.pose.position.x
        posY = data.pose.pose.position.y
        posZ = data.pose.pose.position.z
        self.pose[posX, posY, posZ]
                
    def msgcomd(self, cm_x, cm_y, cm_z):
        twist = Twist()
        twist.linear.x = cm_x
        twist.linear.y = cm_y
        twist.linear.z = cm_z
        self.pubcmd.publish(twist)

    def set_dt(self, dt):
        self.pid_x.setdt(dt)
        self.pid_y.setdt(dt)
        self.pid_z.setdt(dt)

    def set_setpoint(self, sp_x, sp_y, sp_z):
        self.pid_x.setSetPoint(sp_x)
        self.pid_y.setSetPoint(sp_y)
        self.pid_z.setSetPoint(sp_z)

    def controlGoal(self):
        
        dronePose = self.pose

        distance = math.sqrt( (dronePose[0] - self.controller.getSetPoint()[0]) ** 2
                            + (dronePose[1] - self.controller.getSetPoint()[1]) ** 2
                            + (dronePose[2] - self.controller.getSetPoint()[2]) ** 2 )

        self.thresholdList.append(distance)  # Save into the list

        if self.thresholdList > 0:
            average = sum(self.thresholdList) / len(self.thresholdList)

        if (average < self.inReach):
            self.waypointIndex = self.waypointIndex + 1
            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
            self.controller.setSetPoint(self.waypoints[self.waypointIndex])

        # print("X:{x} Y:{y} Z: {r} PSI:{p}".format(
        #     x=dronePosInertial[0], y=dronePosInertial[1], r=dronePosInertial[2], p=np.rad2deg(dronePosInertial[5])))
        # # print("Goal: {g} Average Distance: {a} ".format(g=self.controller.getSetPoint()[0:3], a=average))
       

    def takeoff(self, value):
        if value == 0:
           self.pubLand.publish(Empty())
          
        if value == 1:
           self.pubTakeoff.publish(Empty())  
       
    # def PublishParms(self):
    #     self.pub_pitch.publish(Vector3(self.pitch_pid.kp, 0, self.pitch_pid.kd))
    #     self.pub_roll.publish(Vector3(self.roll_pid.kp ,0, self.roll_pid.kd))
    #     self.pub_alt.publish(Vector3(self.alt_pid.kp, 0, self.alt_pid.kd))

    def DroneModel(self,rect_target):
        
        count=0 # change this for multiple loops over the same path
        while count<1: 
        
            target = rect_target[self.target_count]
            # print(target)
            
            dt = rospy.get_rostime().to_sec() - self.now
            current_time = rospy.get_rostime().to_sec()

            # self.set_dt(dt)
            self.set_setpoint(target[0], target[1], target[2])        
            print(self.posX)
            # self.pid_x.update(self.posX, current_time)
            # self.pid_y.update(self.posY, current_time)
            # self.pid_z.update(self.posZ, current_time)

            # x_input = self.pid_x.output
            # y_input = self.pid_y.output
            # z_input = self.pid_z.output

            print(x_input, y_input, z_input)

            self.msgcomd(x_input, y_input, z_input)
           
           
            self.now = rospy.get_rostime().to_sec()

            print(dt, self.pid_x._error)

            if abs(self.pid_x._error) < 0.1 and abs(self.pid_y._error) < 0.1 and abs(self.pid_z._error) < 0.1:

                self.target_count += 1

                # print "Tuned parameters kp kd for pitch", self.pitch_pid.kp, self.pitch_pid.kd
                # print "Tuned parameters kp kd for roll", self.roll_pid.kp, self.roll_pid.kd
                # print "Tuned parameters kp kd for alt", self.alt_pid.kp,self.alt_pid.kd
                # print ".....Waypoint",self.target_count-1,"Reached"
                # print " ........Chaning Waypoint!"
                print("Cambio de Waypoint")

                self.msgcomd(0, 0, 0)
                rospy.sleep(1)
            if self.target_count == 4:
                self.target_count = 0
                count += 1

            # self.PublishParms()	
        print "Drone Landing"
        # self.pubLand.publish(Empty())
        self.takeoff(0)
if __name__=='__main__':

    rospy.init_node("AutoPID",anonymous=True)
    
    pid = Controller()
    rospy.sleep(1)    
    pid.takeoff(1)
    rospy.sleep(5.0)
    print("despego")
    #while not rospy.is_shutdown():
    rect_target=[[0.0, 0.0, 4.0], [0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [0.0, 0.0, 1.0]] # xy
    print(rect_target)
    rospy.sleep(2)
    #rect_target=[[0,0,1.5],[0,1.5,1.5],[0,1.5,0.8],[0.0,0.0,0.8]] #yz
    pid.DroneModel(rect_target)
