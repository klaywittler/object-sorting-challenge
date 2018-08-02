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

i = 0

#center_x = 639/2
#center_y = 479/2

class MvShape:
    def __init__(self):
        self.contour = None
        self.raw_contour = None
        self.cg = None
        self.color = None
        self.color_lower = None
        self.color_upper = None
        self.area = None
        self.cg_from_center = None

    def __str__(self):
        return str(self.cg) + ", " + str(self.color) + ", " + str(self.area)


def callback(data, args):
    #displaying what was recieved
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print('calling')
    find_color = False
    lower = np.array([0,0,0])
    upper = np.array([0,0,0])
    current_shape = args[0]
    #conditional statement to initiate action
    if data.data == 'track': #track object to go pick up
        #print(current_shape.color_lower)
        lower = current_shape.color_lower
        upper = current_shape.color_upper
        print(lower)
        print(upper)
        find_color = True
    elif data.data == 'found': #move on to next object
        if len(shapes) > 0:
            del args[0]
            #talker()
            print(len(shapes))
            find_color = False
            lower = np.array([0,0,0])
            upper = np.array([0,0,0])
        else:
            print('done')
            #ros.is_shutdown() #command to end ros when done
    talker(find_color, lower, upper)
        

def listener(param):
    rospy.init_node('listener', anonymous=True)

    find = rospy.Subscriber("controller_topic", String, callback,param)

    # spin() simply keeps python from exiting until this node is stopped
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

                return[True, x, y, area, dir]
            break
    return[False, 0, 0, 0, 0]


def talker(find_color = False, lower = np.array([0,0,0]), upper = np.array([0,0,0])):
        #creating publisher
    pub = rospy.Publisher('computerVision_topic', Int32MultiArray, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
        #checks to see if shut down

    error = 1

    if not rospy.is_shutdown():
        if find_color:
            if cap.isOpened(): # try to get the first frame
                rval, frame = cap.read()
                    #center of camera (can be offset for manipulator)
                center_x = frame.shape[1] / 2
                center_y = frame.shape[0] / 2

                [foundTarget,x,y,area,dir] = findTarget(frame, lower, upper)   # Send frame to target track function 
                if foundTarget:
                    #cv2.drawContours(frame, [approx], -1, (0, 255, 0))  # Draw target polygon
                    cv2.circle(frame, (x,y), 2, (255,255,255), -1)      # Draw target center
                        #error from desired center
                    e_x = center_x-x
                    e_y = center_y-y
                        #need to build funtion to figure out bin number
                    box = 1
                    a = [e_x, e_y, box]

                #cv2.imshow("Tracking",frame)
                k = cv2.waitKey(5) & 0xFF
                if k == 27:
                    cv2.destroyAllWindows()
                    cap.release()
                    #break 

                elif not foundTarget:
                    a = [0, 0, -9999]

            elif not cap.isOpened():
                a = [0 , -9999, 0]
        
        elif not find_color:
            a = [-9999, 0, 0]

            #publishing
        pub_a = Int32MultiArray(data=a)
        print('publishing - x: {} y: {} box: {}'.format(a[0], a[1], a[2]))
        #rospy.loginfo(pub_a)
        pub.publish(pub_a)
        rate.sleep() 


def get_shape_information(input_image, min_area = 1000, max_area = 100000, color_bins = 5):
    blur = get_hypersmoothed_image(input_image, 3)
    blur_compact, c = do_kmeans_color_compaction(blur, color_bins)
    # Sorting by color intensity makes testing a bit more consistent
    c = sorted(c, key=lambda c: int(c[0])*int(c[0]) + int(c[1])*int(c[1]) + int(c[2])*int(c[2]))
    imret = blur_compact
    shape_list = list()
    center_x = input_image.shape[1] / 2
    center_y = input_image.shape[0] / 2
    for cl in c:
        c_low, c_up, lower, upper = get_color_bounds_for_threshold(blur_compact, cl, 8)
        ir = cv2.inRange(blur_compact, c_low[0][0], c_up[0][0])
        im2, contours, hierarchy = cv2.findContours(ir, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            s = MvShape()
            s.raw_contour = contour
            s.contour = cv2.approxPolyDP(contour, 15, True)
            s.area = cv2.contourArea(s.contour)
            if (s.area < min_area) or (s.area > max_area):
                continue
            M = cv2.moments(s.contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            s.cg = [cx, cy]
            s.cg_from_center = [cx - center_x, cy - center_y]
            s.color = cl
            s.color_lower = lower
            s.color_upper = upper
            shape_list.append(s)

            imret = cv2.drawContours(imret, [s.contour], -1, (0,255,0), 1, maxLevel = 1)
            imret = cv2.circle(imret, (cx, cy), 2, (255, 0, 0), -1)
            imret = cv2.putText(imret, str(s.cg_from_center), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
    return [shape_list, imret]


def do_kmeans_color_compaction(image, K):
    Z = image.reshape((-1, 3))
    Z = np.float32(Z)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 2.5)
    compactness, labels, centers = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    centers = np.uint8(centers)
    res = centers[labels.flatten()]
    res2 = res.reshape((image.shape))
    return [res2, centers.reshape((-1, 3))]


# Super-smooths out noise in a cluster of polygons, keeping edges
# Works well for our test images
def get_hypersmoothed_image(image, blur_count = 5):
    ii = image
    for i in range(blur_count):
        # 11, 25, 500 good results on test picture
        # 11, 15, 500 better at preserving edges of smaller shapes without too many artifacts
        ii = cv2.bilateralFilter(ii, 11, 15, 500)
    return ii


# Get upper and lower colors for a threshold, given a center color
# and distance from said color. Will compute using HSV space, but
# color input should be RGB.
def get_color_bounds_for_threshold(image, color, dist = 8):
    hsv_col = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)
    hsv_col_high = hsv_col[0][0]
    hsv_col_low = hsv_col[0][0]
    val_high = hsv_col_high[2] + dist
    val_low = hsv_col_low[2] - dist
    if val_high > 255:
        val_high = 255
    if val_low < 0:
        val_low = 0
    hsv_col_high = [hsv_col_high[0], hsv_col_high[1], val_high]
    hsv_col_low  = [hsv_col_low[0],  hsv_col_low[1],  val_low ]
    col_high = cv2.cvtColor(np.uint8([[hsv_col_high]]), cv2.COLOR_HSV2BGR)
    col_low = cv2.cvtColor(np.uint8([[hsv_col_low]]), cv2.COLOR_HSV2BGR)
    return [col_low, col_high, np.array(hsv_col_low),np.array(hsv_col_high)]


def get_map():
    if cap.isOpened():
            rval, frame = cap.read()
            shapes, annot_frame = get_shape_information(frame)

    return [shapes,annot_frame]


if __name__ == '__main__':
    try:
        print('Start Project 2 -- CV')
        cap = cv2.VideoCapture(0)
        shapes, annot_frame = get_map()
        cv2.imwrite("map.png", annot_frame)
        print('map created')
        listener(shapes) #calls listener to find out what to do
        print('End Project 2 -- CV')
    except rospy.ROSInterruptException:
        pass