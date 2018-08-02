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

#number of each color
red = 0
green = 21
blue = 21
yellow = 0
black = 0

#black, blue, red, yellow, green
lower_red = np.array([0,155,200])
upper_red = np.array([10,190,255])

lower_green = np.array([60,130,230])
upper_green = np.array([80,170,255])

lower_blue = np.array([90,200,230])
upper_blue = np.array([120,270,255])

lower_yellow = np.array([20,90,240])
upper_yellow = np.array([40,120,255])

lower_black = np.array([60,70,80])
upper_black = np.array([90,100,120])

#camera image characteristics
width = 639
height = 479

mult_width = 0.55
mult_height = 1


def callback(data):
    #displaying what was recieved
    rospy.loginfo("I heard %s", data.data)

    global red
    global green
    global blue
    global yellow
    global black

    #find_color = True
    if data.data == 'track':
    #calls talker to then find target
        find_color = True
        if not red == 0:
            box = 1
            lower = lower_red
            upper = upper_red
            red -= 1

        elif not green == 0:
            box = 2
            lower = lower_green
            upper = upper_green
            green -= 1 

        elif not blue == 0:
            box = 3
            lower = lower_blue
            upper = upper_blue
            blue -= 1   

        elif not yellow == 0:
            box = 4
            lower = lower_yellow
            upper = upper_yellow
            yellow -= 1
        elif not black == 0:
            box = 5
            find_color = False
            lower = lower_black
            upper = upper_black

        else:
            find_color = False
            lower = np.array([0,0,0])
            upper = np.array([0,0,0])
        talker(find_color, lower, upper, box)
    else:
        find_color = False
        print('invalid command sent.')
    #talker(find_color, lower, upper)


def listener():
    rospy.init_node('cv_node', anonymous=True)

    rospy.Subscriber("controller_topic", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def findTarget(image, lower, upper):
        # Filter image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(hsv,hsv, mask= mask)
    colBlur = cv2.medianBlur(res,7)
    edges = cv2.Canny(colBlur,100,200)
    edBlur = cv2.blur(edges,(13,13))

        # Find contours in filtered image
    cont_image,cont,hierarchy = cv2.findContours(edBlur.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cont = sorted(cont, key = cv2.contourArea, reverse = True)
    if len(cont)>0:
        for c in cont:
            area = cv2.contourArea(c)       # Calcualte area of contour # possibly take some of this out
            if area < 1000 or area > 10000:
                continue
            per = cv2.arcLength(c, True)    # Estimate perimeter of contour
            approx = cv2.approxPolyDP(c, 0.035*per, True)   # Create approximate polygon that best fits contour
            
            area = cv2.contourArea(c)       # Calcualte area of contour # possibly take some of this out
            mom = cv2.moments(approx)       # Calcualte the moments of the polygon
            if mom['m00'] != 0:
            # Calculate center of polygon, x and y position
                x = int(mom['m10']/mom['m00'])
                y = int(mom['m01']/mom['m00'])

                return[True, x, y, area, dir]
            break
    return[False, 0, 0, 0, 0]


def talker(find_color=False, lower=np.array([0,0,0]), upper=np.array([0,0,0]), box = 0):
    #creating publisher
    pub = rospy.Publisher('computerVision_topic', Int32MultiArray, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
        #checks to see if shut down

    #tracking variables
    i = 6
    target_hold = 5
    e_x = 0
    e_y = 0
    error = 35
    lower_thresh = 40
    upper_thresh = 80

    while True: 
        if not rospy.is_shutdown():
            if find_color:
                if cap.isOpened(): # try to get the first frame
                    rval, frame = cap.read()
                        #center of camera (can be offset for manipulator)
                    frame = frame[0:int(mult_height*frame.shape[0]),0:int(mult_width*frame.shape[1])]
                    center_x = 0.95*frame.shape[1]+40
                    center_y = frame.shape[0]/2-20

                    [foundTarget,x,y,area,dir] = findTarget(frame, lower, upper)   # Send frame to target track function 
                    if foundTarget:
                        #cv2.drawContours(frame, [approx], -1, (0, 255, 0))  # Draw target polygon
                        cv2.circle(frame, (x,y), 2, (0,255,0), -1)      # Draw target center
                            #error from desired center
                        e_x = x-center_x
                        e_y = y-center_y

                        error = math.sqrt((e_x)**2 + (e_y)**2)
                        #print(error)
                        print("error: {}".format(error))
                            #need to build funtion to figure out bin number                        
                        a = [e_x, e_y, 0]

                    cv2.imshow("Tracking",frame)
                    k = cv2.waitKey(5) & 0xFF
                    if k == 27:
                        cv2.destroyAllWindows()
                        cap.release()
                        #break 

                    elif not foundTarget:
                        a = [0, 0, -9999]

                elif not cap.isOpened():
                    #a = [0 , -9999, 0]
                    pass
                    break
            elif not find_color:
                #a = [-9999, 0, 0]
                pass
                break

                #publishing
            if error < lower_thresh:
                i += 1
                if i > target_hold:
                    a = [0, 0, box]
                    pub_a = Int32MultiArray(data=a)
                    #rospy.loginfo(pub_a) 
                    print('final publish - ex: {} ey: {} box: {}'.format(0, 0, box))
                    pub.publish(pub_a)
                    rate.sleep()
                    break
            elif error > upper_thresh and i > 0:
                i -= 1
                if i < 0:
                    i = 0

            #publishing
            pub_a = Int32MultiArray(data=a)
            print('publish - ex: {} ey: {} box: {}'.format(e_x, e_y, 0))
            #rospy.loginfo(pub_a)
            pub.publish(pub_a)
            rate.sleep()

    print('target aquired') 


if __name__ == '__main__':
    try:
        print('Start Project 2 -- CV')
        cap = cv2.VideoCapture(1) #open video - 0 for webcam, 1 for usb camera
        listener() 
        print('End Project 2  -- CV')
    except rospy.ROSInterruptException:
        pass