#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import time 
import math

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg

#black, blue, red, yellow, green
lower_red = np.array([30,150,50])
upper_red = np.array([255,255,180])

lower_green = np.array([50,100,100])
upper_green = np.array([70,255,255])

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

lower_yellow = np.array([0,100,150])
upper_yellow = np.array([25,255,255])

width = 639
height = 479

center_x = 639/2
center_y = 479/2

def callback(data):
    #displaying what was recieved
    rospy.loginfo("I heard %s", data.data)
    #find_color = True
    lower = lower_red
    upper = upper_red
    if data.data == 'not found':
    #calls talker to then find target
        find_color = True
    elif data.data == 'found':
        find_color = False
    return find_color

def listener():
    rospy.init_node('cv_node', anonymous=True)

    find_color = rospy.Subscriber("controller_topic", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    talker(find_color, lower_red, upper_red)
    rospy.spin()

def findTarget(image, lower, upper):
        # Filter image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(hsv,hsv, mask= mask)
    colBlur = cv2.medianBlur(res,7)
    edges = cv2.Canny(colBlur,100,200)
    edBlur = cv2.blur(edges,(50,50))

        # Find contours in filtered image
    cont_image,cont,hierarchy = cv2.findContours(edBlur.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cont = sorted(cont, key = cv2.contourArea, reverse = True)
    if len(cont)>0:
        for c in cont:
            per = cv2.arcLength(c, True)    # Estimate perimeter of contour
            approx = cv2.approxPolyDP(c, 0.035*per, True)   # Create approximate polygon that best fits contour
            
            area = cv2.contourArea(c)       # Calcualte area of contour # possibly take some of this out
            mom = cv2.moments(approx)       # Calcualte the moments of the polygon
            if mom['m00'] != 0:
            # Calculate center of polygon, x and y position
                x = int(mom['m10']/mom['m00'])
                y = int(mom['m01']/mom['m00'])

                return[True, x, y, area, approx, dir]
            break
    return[False, 0, 0, 0, 0, 0]

def talker(find_color, lower, upper):
    #print('talking...')
    pub = rospy.Publisher('computerVision_topic', Int32MultiArray, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if find_color:
            if cap.isOpened(): # try to get the first frame
                rval, frame = cap.read()
            
                [foundTarget,x,y,area,approx,dir] = findTarget(frame, lower, upper)   # Send frame to target track function 
                if foundTarget:
                    cv2.drawContours(frame, [approx], -1, (0, 255, 0))  # Draw target polygon
                    cv2.circle(frame, (x,y), 2, (255,255,255), -1)      # Draw target center
                    e_x = center_x-x
                    e_y = center_y-y
                    a = [e_x, e_y]
                    pub_a = Int32MultiArray(data=a)
                    print('publishing - x: {} y: {}'.format(e_x, e_y))
                    #rospy.loginfo(pub_a)
                    pub.publish(pub_a)
                    rate.sleep()
                elif not foundTarget:
                    a = [-9999, rospy.get_time()]
                    pub_a = Int32MultiArray(data=a)

                    rospy.loginfo(pub_a)
                    pub.publish(pub_a)
                    rate.sleep()

            elif not cap.isOpened():
                a = [0 , rospy.get_time()]
                pub_a = Int32MultiArray(data=a)

                rospy.loginfo(pub_a)
                pub.publish(pub_a)
                rate.sleep()
        
            cv2.imshow("Image",frame)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                cap.release()
                break 
        elif not find_color:
            a = [rospy.get_time(), -9999]
            pub_a = Int32MultiArray(data=a)

            rospy.loginfo(pub_a)
            pub.publish(pub_a)
            rate.sleep()       
    rospy.spin()

if __name__ == '__main__':
    try:
        print('Start Project 2 -- CV')
        cap = cv2.VideoCapture(0) #open video - 0 for webcam, 1 for usb camera
        listener() 
        print('End Project 2  -- CV')
    except rospy.ROSInterruptException:
        pass