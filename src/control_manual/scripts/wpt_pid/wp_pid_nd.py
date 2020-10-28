#!/usr/bin/env python

import rospy
# from tum_ardrone.msg import filter_state
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty, Float64
# from PIDAutoTuning import PIDAutoTuning
from nav_msgs.msg import Odometry   # linea nueva

from PID import PID
import numpy as np
import tf
import math
import collections
import csv

from time import time, strftime

class Controller:

    def __init__(self):
        
        #----- Modifiable Parameters
        # PID controller
        # Proportional parameters XYZrpy
        kp = np.array([0.4, 0.4, 1.0, 0, 0, 1])     # kp = np.array([0.15, 0.15, 0.2, 0, 0, 0.15])
        # Integral parameters XYZrpy
        ki = np.array([0.11, 0.11, 0.11, 0, 0, 0.11])    # ki = np.array([0.01, 0.01, 0.01, 0, 0, 0.01]) 
        # Proportional parameters XYZrpy
        kd = np.array([0.35, 0.36, 0.0, 0, 0, 0.0])

        samplingTime = 0.1  # 100 [ms]

        # self.waypoints = [np.array([0, 0, 2, 0, 0, 0]),
        #                   np.array([0, 0, 2, 0, 0, 0.79]),
        #                   np.array([0, 0, 2, 0, 0, 1.57]),
        #                   np.array([0, 0, 2, 0, 0, 2.36]),
        #                   np.array([0, 0, 2, 0, 0, 3.14]),
        #                   np.array([0, 0, 2, 0, 0, 3.93])]
                         

        # self.waypoints = [np.array([0, 0, 2, 0, 0, -1.54]),
        #                   np.array([0, 0, 2, 0, 0, -1.54]),
        #                   np.array([1, 0, 2, 0, 0, -1.54]),
        #                   np.array([2, 0, 2, 0, 0, -1.54]),
        #                   np.array([3, 0, 2, 0, 0, -1.54]),
        #                   np.array([4, 0, 2, 0, 0, -1.54]),
        #                   np.array([4, 1, 2, 0, 0, -1.54]),
        #                   np.array([4, 2, 2, 0, 0, -1.54]),
        #                   np.array([4, 3, 2, 0, 0, -1.54]),
        #                   np.array([4, 4, 2, 0, 0, -1.54]),
        #                   np.array([3, 4, 2, 0, 0, -1.54]),
        #                   np.array([2, 4, 2, 0, 0, -1.54]),
        #                   np.array([1, 4, 2, 0, 0, -1.54]),
        #                   np.array([0, 4, 2, 0, 0, -1.54]),
        #                   np.array([0, 3, 2, 0, 0, -1.54]),
        #                   np.array([0, 2, 2, 0, 0, -1.54]),
        #                   np.array([0, 1, 2, 0, 0, -1.54]),
        #                   np.array([0, 0, 2, 0, 0, -1.54])]

    
        self.waypoints = [np.array([0.0, 0.0, 1.25, 0, 0, -1.54]),
                          np.array([1.0 ,0.0, 1.25, 0, 0, -1.54]),
                          np.array([1.0, 1.0, 1.25, 0, 0, -1.54]),
                          np.array([0.0, 1.0, 1.25, 0, 0, -1.54]),
                          np.array([0.6, 0.0, 1.25, 0, 0, -1.54])]
        
        # operate at 5 Hz because that is what we get from odometry and the 
        # drone needs commands refreshed otherwise it will just hold position                
        self.freqTopic = 10  # 10 Frequency of topic messages

        self.stableTime = 5  # Time in the vecinity of target waypoint 5[s]
        self.inReach = 0.1    # Distance to be considered in the vecinity of the waypoint       

        # ----- Initialization stage
        self.controller = PID()  # controller
        self.controller.setKp(kp)
        self.controller.setKi(ki)
        self.controller.setKd(kd)

    
        np.set_printoptions(precision=3) # to print numpy

        self.controller.setSamplingTime(samplingTime)

        self.waypointIndex = 0      # Index to track current waypoints
        self.controller.setSetPoint(self.waypoints[self.waypointIndex])

        

        # Numero de datos almacenados en lista, 
        self.thresholdList = collections.deque(maxlen=(self.stableTime*self.freqTopic))  # 50 
        

        # Create logfile named by current date / time
        # filename = 'pruebas_bebop/trajecto' + strftime('%d_%b_%Y_%H_%M_%S') + '.csv'
        # self.logfile = open(filename, 'wb')
        # self.write_and_flush('X pose [m], Y pose [m], Z pose [m]\n')


        self.pidx = 0
        self.pidy = 0
        self.pidz = 0
        self.pidyaw = 0
        self.eldato = 0

        # self.pub_X = 0
        # self.pub_Y = 0
        # self.pub_Z = 0
        # self.pub_Yaw = 0


        # Inicializacion de nodo
        rospy.init_node("Bebop_WP_PID",anonymous=True)  
        self.rate = rospy.Rate(self.freqTopic)
        

        self.pubTakeoff = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.pubLand = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.pubcmd = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)


        #  # Publicar setpoint para el nod PID
        self.pub_X = rospy.Publisher('/setpoint_X', Float64, queue_size=10)     
        self.pub_Y = rospy.Publisher('/setpoint_Y', Float64, queue_size=10)
        self.pub_Z = rospy.Publisher('/setpoint_Z', Float64, queue_size=10)
        self.pub_Yaw = rospy.Publisher('/setpoint_Yaw', Float64, queue_size=10)
        
        
        
        # Input
        self.pub_X_o = rospy.Publisher('/pos_X/state', Float64, queue_size=10)     
        self.pub_Y_o = rospy.Publisher('/pos_Y/state', Float64, queue_size=10)
        self.pub_Z_o = rospy.Publisher('/pos_Z/state', Float64, queue_size=10)
        self.pub_Yaw_o = rospy.Publisher('/pos_Yaw/state', Float64, queue_size=10)

        # Output
        self.pid_X = rospy.Subscriber("/pos_X/control_effort", Float64, self.subpidX)
        self.pid_Y = rospy.Subscriber("/pos_Y/control_effort", Float64, self.subpidY)
        self.pid_Z = rospy.Subscriber("/pos_Z/control_effort", Float64, self.subpidZ)
        self.pid_Yaw = rospy.Subscriber("/pos_Yaw/control_effort", Float64, self.subpidYAW)
        


        

        initialOdom = rospy.wait_for_message('/bebop/odom', Odometry)
        initialInertialCoord = self.odometryMsg2InertialCoordinates(initialOdom)

        # Actualiza wayPoint y verifica si la meta se ha alcanzado
        self.poseForPathSubscriber = rospy.Subscriber('bebop/odom', Odometry, self.controlGoal)
         
        # Despegue 0:land, 1:takeoff
        # self.takeoff(1)
        rospy.sleep(5)

        while not rospy.is_shutdown():
            self.poseForControlSubscriber = rospy.Subscriber("/bebop/odom", Odometry, self.bebopControl)

            self.pub_X_o = rospy.Publisher('/pos_X/state', Float64, queue_size=10)     
            self.pub_Y_o = rospy.Publisher('/pos_Y/state', Float64, queue_size=10)
            self.pub_Z_o = rospy.Publisher('/pos_Z/state', Float64, queue_size=10)
            self.pub_Yaw_o = rospy.Publisher('/pos_Yaw/state', Float64, queue_size=10)

            # Output
            self.pid_X = rospy.Subscriber("/pos_X/control_effort", Float64, self.subpidX)
            self.pid_Y = rospy.Subscriber("/pos_Y/control_effort", Float64, self.subpidY)
            self.pid_Z = rospy.Subscriber("/pos_Z/control_effort", Float64, self.subpidZ)
            self.pid_Yaw = rospy.Subscriber("/pos_Yaw/control_effort", Float64, self.subpidYAW)
            self.rate.sleep()

        
    def bebopControl(self, data):

        dronePosInertial = self.odometryMsg2InertialCoordinates(data)

        accion = self.controller.controll(dronePosInertial, rospy.Time.now().to_time())
        
        
        self.twist = Twist()
        self.twist.linear.x = self.pid_X
        self.twist.linear.y = self.pid_Y
        self.twist.linear.z = self.pid_Z
        self.twist.angular.z = self.pid_Yaw

        self.pubcmd.publish(self.twist)
            # print(self.twist)


    def controlGoal(self, message):

        
       
        dronePose = self.odometryMsg2InertialCoordinates(message)

        self.pub_X_o = dronePose[0]  
        self.pub_Y_o = dronePose[1]  
        self.pub_Z_o = dronePose[2]  
        self.pub_Yaw_o = dronePose[5]  

        distance = math.sqrt( (dronePose[0] - self.controller.getSetPoint()[0]) ** 2
                            + (dronePose[1] - self.controller.getSetPoint()[1]) ** 2
                            + (dronePose[2] - self.controller.getSetPoint()[2]) ** 2 )
        # self.pub_X.publish(distance)

        self.thresholdList.append(distance)  # Save into the list, 50 datos maximo

        if self.thresholdList > 0:
            average = sum(self.thresholdList) / len(self.thresholdList)

        # si el promedio es menor a 0.1 pasa al siguiente waypoint
        if (average < self.inReach):
            print("true")
            self.waypointIndex = self.waypointIndex + 1

            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
                # TODO agregar el LAND aca           
            # self.controller.clear()
            self.controller.setSetPoint(self.waypoints[self.waypointIndex])

            # setpoint's
            waypoint = self.waypoints[self.waypointIndex]
            rospy.loginfo(waypoint)
            # self.pub_X.publish(self.controller.getSetPoint()[0])
            # self.pub_Y.publish(self.controller.getSetPoint()[1])
            # self.pub_Z.publish(self.controller.getSetPoint()[2])
            # self.pub_Yaw.publish(self.controller.getSetPoint()[5])
            
                             

        # self.write_and_flush('%3.3f, %3.3f, %3.3f'\n % (dronePose[0], dronePose[1], dronePose[2]))


    def takeoff(self, value):

        if value == 0:
           self.pubLand.publish(Empty())          
        if value == 1:
           self.pubTakeoff.publish(Empty()) 
        rospy.sleep(5)


    def write_and_flush(self, s):
        self.logfile.write(s)
        self.logfile.flush()

    def subpidX(self, data):
        self.pid_X = data.data

    def subpidY(self, data):
        self.pid_Y = data.data


    def subpidZ(self, data):
        self.pid_Z = data.data
        print(self.pid_Z)
    
    def subpidYAW(self, data):
        self.pid_Yaw = data.data
       
        
    @staticmethod
    def odometryMsg2InertialCoordinates(message):
        """Convierte el mensaje Odometria coordenadas inerciales (x, y, z, phi, theta, psi)
        """
        # Get the pose from the message
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
        # print("1")

       
        return dronePosInertialWAng


if __name__=='__main__':   

    try:  
        trajectoria = Controller()  
        print("Controller")
        
    except rospy.ROSInterruptException:
        print("Error!!!")


   