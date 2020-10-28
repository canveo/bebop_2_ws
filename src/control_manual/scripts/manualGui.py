#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5 import QtCore, QtGui, uic, QtWidgets
from PyQt5.QtGui import QKeySequence   
from Ui_main import Ui_Form
import sys

#import roslib; roslib.load_manifest('control_manual_node')
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import threading
import time

from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as Battery

class Ui(QtWidgets.QWidget, Ui_Form):

    def __init__(self, parent=None):
        super(Ui, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.show()
        # acción botones
        self.ui.Avanza.clicked.connect(self.avanza)
        self.ui.Retrocede.clicked.connect(self.retrocede)
        self.ui.Derecha.clicked.connect(self.derecha)
        self.ui.Izquierda.clicked.connect(self.izquierda)
        self.ui.takeoff_land.valueChanged.connect(self.takeoff)
        # self.ui.Stop.clicked.connect(self.get_yaw)
        self.ui.Sube.clicked.connect(self.arriba)
        self.ui.Baja.clicked.connect(self.abajo)
        self.ui.GiraDer.clicked.connect(self.giraDerecha)
        self.ui.GiraIzq.clicked.connect(self.giraIzquierda)
        self.ui.Bateria.setValue(0)
        self.ui.Emergencia.clicked.connect(self.emergencia)
        self.ui.trayectorias.currentIndexChanged.connect(self.trajectorySelect)
        self.ui.Enviar.clicked.connect(self.publica)

        
        # ROS
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.emer_pub = rospy.Publisher("/bebop/reset", Empty, queue_size=10)

        self.opt = 0

    def takeoff(self, value):
        if value == 0:
           self.land_pub.publish(Empty())
          
        if value == 1:
           self.takeoff_pub.publish(Empty())            

    def avanza(self):
        twist = Twist()
        twist.linear.x = 0.5
        self.vel_pub.publish(twist)

    def retrocede(self):
        twist = Twist()
        twist.linear.x = -0.5
        self.vel_pub.publish(twist)
    
    def derecha(self):
        twist = Twist()
        twist.linear.y = -0.5
        self.vel_pub.publish(twist)
    
    def izquierda(self):
        twist = Twist()
        twist.linear.y = 0.5
        self.vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        self.vel_pub.publish(twist)

    
    def giraDerecha(self):
        twist = Twist()
        twist.angular.z = -0.5
        self.vel_pub.publish(twist)

    def giraIzquierda(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.vel_pub.publish(twist)

    def arriba(self):
        twist = Twist()
        twist.linear.z = 0.5
        self.vel_pub.publish(twist)

    def abajo(self):
        twist = Twist()
        twist.linear.z = -0.5
        self.vel_pub.publish(twist)

    
    def emergencia(self):
        self.emer_pub.publish(Empty())
    
    def bateria(self, data):
        self.ui.Bateria.setValue(data.percent)
        # for i in range(0, 100, 1):
        #     self.ui.Bateria.setValue(i) 
        #     time.sleep(1)   
        # pass

    def listener(self):
        rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", Battery, self.bateria) 
        # rospy.spin()    

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space:
            self.emergencia()

    def trajectorySelect(self, opt):
        self.opt = opt
        # print(opt)

    def publica(self):
        print(self.opt)

        
if __name__ == '__main__':  
     
    rospy.init_node('control_manual_node', anonymous=True)
    app = QtWidgets.QApplication(sys.argv)
    window = Ui() 
    # window.listener()
    t = threading.Thread(target=window.listener())
    t.start()
    window.show()
    sys.exit(app.exec_())
    
    

