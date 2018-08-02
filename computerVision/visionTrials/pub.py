#!/usr/bin/env python
# license removed for brevity

import cv2
import numpy as np
import time 
import math

import rospy
#from std_msgs.msg import Int32MultiArrary
from std_msgs.msg import String

#black, blue, red, yellow, green
lower_red = np.array([30,150,50])
upper_red = np.array([255,255,180])

lower_green = np.array([50,100,100])
upper_green = np.array([70,255,255])

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

lower_yellow = np.array([0,100,150])
lower_yellow = np.array([25,255,255])


width = 320
height = 120

cap = cv2.VideoCapture(0)

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

                #l = math.sqrt(area)
                #ymin = int(max(y-l/2,0))
                #ymax = int(min(y+l/2, height-1))
                #xmin = int(max(x-l, 0))
                #xmax = int(min(x+l, width-1))
                #hsv_image_clipped = hsv[ymin:ymax, xmin:xmax,:]

                return[True, x, y, area, approx, dir]
            break
    return[False, 0, 0, 0, 0, 0]

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if cap.isOpened(): # try to get the first frame
            #black, blue, red, yellow, green

            rval, frame = cap.read()
        
            [foundTarget,x,y,area,approx,dir] = findTarget(frame, lower_blue, upper_blue)   # Send frame to target track function 
            if foundTarget:
                cv2.drawContours(frame, [approx], -1, (0, 255, 0))  # Draw target polygon
                cv2.circle(frame, (x,y), 2, (255,255,255), -1)      # Draw target center

                #a = [x,y]
                #pub_a = Int32MultiArrary(data=a)
                pub_a = "(" + str(x) + ", " + str(y) + ")"

                rospy.loginfo(pub_a)
                pub.publish(pub_a)
                rate.sleep()
            else:

                #a = [-9999, rospy.get_time()]
                #pub_a = Int32MultiArrary(data=a)
                pub_a = "Target not found. "

                rospy.loginfo(pub_a)
                pub.publish(pub_a)
                rate.sleep()

        else:

            #a = [0 , rospy.get_time()]
            #pub_a = Int32MultiArrary(data=a)
            pub_a = "Video not opened. "

            rospy.loginfo(pub_a)
            pub.publish(pub_a)
            rate.sleep()
        
        cv2.imshow("Image",frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

if __name__ == '__main__':
    try:
        talker()
        cv2.destroyAllWindows()
        cap.release()
    except rospy.ROSInterruptException:
        pass