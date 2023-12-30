#!/usr/bin/env python2

import sys
import numpy as np
import cv2
import time
import datetime

from pymycobot import MyCobot
import rospy
from std_msgs.msg import Bool





def camera():
    
    delay_move = 3.0
    
    
    
    """ROS"""
    pub = rospy.Publisher('move_enable', Bool, queue_size=10)
    rospy.init_node('camera_node')
    rate = rospy.Rate(10)
    
    
    
    """Camera capture."""

    cap = cv2.VideoCapture(0)

    # square
    vert = [None] * 5
    vert[0] = (400, 500)
    vert[1] = (700, 500)
    vert[2] = (700, 200)
    vert[3] = (400, 200)
    vert[4] = (400, 500)


    tm_start = 0
    tm_inside = 0
    
    start_time = time.time()

    while True:
        success, frame = cap.read()
        if not success:
            print("Cannot connect to the camera.")
            break
        resize = cv2.resize(frame, (1000, 720))

        for i in range(len(vert) - 1):
            cv2.line(resize, vert[i], vert[(i + 1)], (0, 255, 0), 2)

        green_contours = cv2.inRange(resize, (0, 255, 0), (0, 255, 0))
        green_contours, hierarchy = cv2.findContours(green_contours.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        cv2.drawContours(resize, green_contours, -1, (0, 0, 255), 1)

        gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 9)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=100, param2=30,
                                   minRadius=80, maxRadius=200)  # adjust to the height of the camera

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                #print(center)
                cv2.circle(resize, center, 1, (0, 100, 100), 3)
                radius = i[2]
                cv2.circle(resize, center, radius, (255, 0, 255), 3)

                if tm_inside - tm_start >= 2 and tm_start != 0:
                    print("Ball ready to be extracted!")
                
                
                if center[0] - radius > vert[0][0] and center[0] + radius < vert[1][0] and center[1] + radius < vert[0][1] and center[1] - radius > vert[2][1]:
                    elapsed_time = time.time() - start_time
                    print("I see a ball")
                    if elapsed_time > delay_move:
                        print('sendend message')
                        start_time = time.time()
                        pub.publish(True)
                else:
                    start_time = time.time()
                    elapsed_time = time.time()
                    

                if cv2.waitKey(10) == ord('r'):
                    tm_start = 0
                    tm_inside =0

        cv2.imshow("camera capture", resize)

        if cv2.waitKey(10) == ord('q'):
            break


if __name__ == '__main__':
    
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
 
 




