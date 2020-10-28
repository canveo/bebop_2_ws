#!/usr/bin/env python
#
# title           :hw04_task01.py
# description     :Program to fly with a specified yaw angle in each line using
#                  a PID controller and visiting a sequence of waypoints.
# author          :Carlos Hansen
# date            :28-11-2019
# pythonVersion   :2.7.15
# ==============================================================================
from PID import PID
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf                   # transformation between euler and quaternions
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg    # for the manipulation of messages to comand the robot

import math
import collections          # for having a fixed size list for the stability

import csv                  # for saving data into a csv file


class OdometryDrone:
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

        # Initialize the node
        rospy.init_node('drone_controller', anonymous=True)
        self.rate = rospy.Rate(self.freqTopic)  # 10hz

        # Initialize CV Bridge
        # self.bridge = CvBridge()

        # # Subscriber to retrieve raw ROS image
        # self.imageSub = rospy.Subscriber('/bebop/image_raw',Image, self.imageCallback)

        # #Publisher for the IMG
        # self.image_pub = rospy.Publisher("image_topic_2",Image)

        # To control the position
        self.cmdMsg = geometry_msgs.msg.Twist()

        # Publisher to command the robot via the topic /cmd_vel
        self.cmdPub = rospy.Publisher(
            '/bebop/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # ------ Getting the initial parameters of the drone
        initialOdom = rospy.wait_for_message(
            'bebop/odom', nav_msgs.msg.Odometry,)
        initialInertialCoord = self.odometryMsg2InertialCoordinates(
            initialOdom)

        # angle between body and bebop orientation
        self.bebopAngled = initialInertialCoord[3]

        # ------ Visualization of trajectories for RVIZ
        self.rvizMsg = nav_msgs.msg.Odometry()
        self.rvizHeader = std_msgs.msg.Header()
        self.rvizHeader.frame_id = "/odom"

        # Publisher for visualization
        self.rvizPub = rospy.Publisher(
            'drone_odometry', nav_msgs.msg.Odometry, queue_size=10)

        # Subscriber to check for every pose into a messge for rviz
        self.rvizSubscriber = rospy.Subscriber(
            'bebop/odom', nav_msgs.msg.Odometry, self.robotPosOr)

        # ------ Update target
        # Subscriber to check if the corner had been reach and change the goal corner to go
        self.poseForPathSubscriber = rospy.Subscriber(
            'bebop/odom', nav_msgs.msg.Odometry, self.controlGoal)

        # ------ Drone commands

        # Takeoff of the drone
        self.takeoff(5)

        # every time the odometry filtered is received the control parameters get updated
        # Initialize time of controller
        while not rospy.is_shutdown():
            # print("----")
            self.poseForControlSubscriber = rospy.Subscriber(
                'bebop/odom', nav_msgs.msg.Odometry, self.droneController)
            self.rate.sleep()

        # Land
        # self.land(10)

    def takeoff(self, takeoffTime):
        """ Publication of the message to takeoff during the specified time
        
        @param takeoffTime: Time during which the topic will be publish
        @type time: Time_obj 
        """
        print("Takingoff")

        takeoffPub = rospy.Publisher(
            "bebop/takeoff", std_msgs.msg.Empty, queue_size=1)

        beginTime = rospy.Time.now()  # Starting time
        #note to work from rospy.Time a python time its a build idnfunc rospy.Time.to_time(beginTime)
        # Desired duration in terms of ros
        secondsTakeoff = rospy.Duration(takeoffTime)
        # Desired duration in reference of begininig
        endTime = secondsTakeoff + beginTime
        while rospy.Time.now() < endTime:  # publication of message while duration
            takeoffPub.publish(std_msgs.msg.Empty())
            self.rate.sleep()

    def land(self, landTime):
        """ Publication of the message to land during the specified time
        
        @param takeoffTime: Time during which the topic will be publish
        @type time: Time_obj 
        """
        print("Landing")

        landPub = rospy.Publisher(
            "bebop/land", std_msgs.msg.Empty, queue_size=1)

        beginTime = rospy.Time.now()  # Starting time

        # Desired duration in terms of ros
        secondsTakeoff = rospy.Duration(landTime)
        # Desired duration in reference of begininig
        endTime = secondsTakeoff + beginTime
        while rospy.Time.now() < endTime:  # publication of message while duration
            landPub.publish(std_msgs.msg.Empty())
            self.rate.sleep()

    def droneController(self, message):
        """"Control the drone"""

        dronePosInertial = self.odometryMsg2InertialCoordinates(message)
        # print(dronePosInertial)

        ctrlOutput = self.controller.controll(dronePosInertial, rospy.Time.now().to_time())

        # Create command

        if ctrlOutput is not None :
            self.cmdMsg.linear.x = ctrlOutput[0]
            self.cmdMsg.linear.y = ctrlOutput[1]
            self.cmdMsg.linear.z = ctrlOutput[2]
            self.cmdMsg.angular.z = ctrlOutput[5]

            # Publish
            self.cmdPub.publish(self.cmdMsg)

    def controlGoal(self, message):
        """Measure the current distance to the goal and if reached, update the next goal. Then save it to an CSV file"""

        dronePosInertial = self.odometryMsg2InertialCoordinates(message)

        distance = math.sqrt((dronePosInertial[0]-self.controller.getSetPoint()[0]) ** 2
                             + (dronePosInertial[1]-self.controller.getSetPoint()[1]) ** 2
                             + (dronePosInertial[2]-self.controller.getSetPoint()[2]) ** 2)

        self.thresholdList.append(distance)  # Save into the list

        if self.thresholdList > 0:
            average = sum(self.thresholdList) / len(self.thresholdList)

        if (average < self.inReach):
            self.waypointIndex = self.waypointIndex + 1
            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
            self.controller.setSetPoint(self.waypoints[self.waypointIndex])

        print("X:{x} Y:{y} Z: {r} PSI:{p}".format(
            x=dronePosInertial[0], y=dronePosInertial[1], r=dronePosInertial[2], p=np.rad2deg(dronePosInertial[5])))
        # print("Goal: {g} Average Distance: {a} ".format(g=self.controller.getSetPoint()[0:3], a=average))
       
        # Saving to file

        dataToSave = np.append(dronePosInertial, distance)
        print(dataToSave)

        with open(self.fileName, mode='a') as data_file:
            file_writter = csv.writer(
                data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            file_writter.writerow(dataToSave)

    def robotPosOr(self, message):
        """Visualization of the odometry nav msgs/Odometry"""
        dronePosInertial = self.odometryMsg2InertialCoordinates(message)

        # constructing poses part of the message
        self.rvizMsg.pose.pose.position.x = dronePosInertial[0]
        self.rvizMsg.pose.pose.position.y = dronePosInertial[1]
        self.rvizMsg.pose.pose.position.z = dronePosInertial[2]

        # Orientations into quaternions
        quat = tf.transformations.quaternion_from_euler(
            0, 0, dronePosInertial[5], 'ryxz')

        # constructing orientation part of the message
        self.rvizMsg.pose.pose.orientation.x = quat[0]
        self.rvizMsg.pose.pose.orientation.y = quat[1]
        self.rvizMsg.pose.pose.orientation.z = quat[2]
        self.rvizMsg.pose.pose.orientation.w = quat[3]

        # Update of the Header
        self.rvizHeader.stamp = rospy.Time.now()
        self.rvizMsg.header = self.rvizHeader

        # Publish
        self.rvizPub.publish(self.rvizMsg)

    def imageCallback(self, msg):
        # image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    @staticmethod
    def odometryMsg2InertialCoordinates(message):
        """Convert an Odometry message to Inertial Coordinates (x, y, z, phi, theta, psi)
        
        @param takeoffTime: Time during which the topic will be publish
        @type time: Time_obj 
        @returns: Inertial coordinates
        @rtype: numpy.ndarray(1_6) 
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

        return dronePosInertialWAng


########################################
if __name__ == "__main__":
    try:
        task01 = OdometryDrone()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
