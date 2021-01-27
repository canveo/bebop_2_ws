#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np


def filter_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv image", hsv)

    #find the upper and lower bounds the yellow color
    yellowLower = (17, 139, 166)
    yellowUpper = (37, 159, 246)

    # array([ 17, 139, 166]), array([ 37, 159, 246]))

    #define a mask using the lower andthe upper bounds of the yellow color
    mask = cv2.inRange(hsv, yellowLower, yellowUpper)

    cv2.imshow("mask image", mask)
    return mask

def getContours(binary_image):      
    _, contours, _ = cv2.findContours(binary_image, 
                                              cv2.RETR_TREE, 
                                               cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contours(image, contours, image_name):
    index = -1 
    thickness = 2
    color = (0, 0, 255) 
    cv2.drawContours(image, contours, index, color, thickness)
    cv2.imshow(image_name,image)

def convert_gray_to_binary(gray_image):
    binary_image = cv2.adaptiveThreshold(gray_image, 
                            255, 
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                            cv2.THRESH_BINARY_INV, 115, 2)
    
    return binary_image

def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>3000):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            # print ("Area: {}, Perimeter: {}".format(area, perimeter))
    # print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy


def ball_detected(frame):
    mask = filter_color(frame)
    contours = getContours(mask)
    draw_ball_contour(mask, frame, contours)
    cv2.imshow('Mask', mask)
    

def main(args):
    u_sub = usbcam_tracker()
    rospy.init_node("Video_subscriber", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shuting down")
    cv2.destroyAllWindows()


class usbcam_tracker:
    def __init__(self):
        self.video_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        source = 0
        self.cap = cv2.VideoCapture(source)

    def callback(self, data):
        try:
            ret, frame = self.cap.read()
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            mask = filter_color(frame)
            contours = getContours(mask)
            draw_ball_contour(mask, frame, contours)
            cv2.imshow("Image2", mask)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

if __name__== "__main__":
    main(sys.argv)
