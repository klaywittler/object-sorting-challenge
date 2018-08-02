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
max_area = 10000
min_area = 1000
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
        self.box = None

    def __str__(self):
        return str(self.cg) + ", " + str(self.color) + ", " + str(self.area)


def callback(data, args):
    #displaying what was recieved
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print('calling')
    find_color = False
    #lower = np.array([0,0,0])
    #upper = np.array([0,0,0])
    current_shape = args[0]
    #box = 0
    #conditional statement to initiate action
    if data.data == 'track': #track object to go pick up
        #print(current_shape.color_lower)
        #lower = current_shape.color_lower
        #upper = current_shape.color_upper
        #box = current_shape.box
        
        find_color = True
        talker(current_shape, find_color)
    elif data.data == 'found': #move on to next object
        if len(shapes) > 0:
            del args[0]
            #talker()
            print('Length of shapes: {}'.format(len(shapes)))
            #find_color = False
            #lower = np.array([0,0,0])
            #upper = np.array([0,0,0])
        else:
            print('done')
            #ros.is_shutdown() #command to end ros when done


def listener(param):
    rospy.init_node('cv_node', anonymous=True)

    find = rospy.Subscriber("controller_topic", String, callback, param)

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
            area = cv2.contourArea(c)       # Calcualte area of contour # possibly take some of this out
            if area < min_area or area > max_area:
                continue

            per = cv2.arcLength(c, True)    # Estimate perimeter of contour
            approx = cv2.approxPolyDP(c, 0.035*per, True)   # Create approximate polygon that best fits contour

            mom = cv2.moments(approx)       # Calcualte the moments of the polygon
            if mom['m00'] != 0:
            # Calculate center of polygon, x and y position
                x = int(mom['m10']/mom['m00'])
                y = int(mom['m01']/mom['m00'])

                return[True, x, y, area, dir]
            break
    return[False, 0, 0, 0, 0]


def talker(shape, find_color = False): #(find_color = False, lower = np.array([0,0,0]), upper = np.array([0,0,0]), b = 0):
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
                    #fgbg = cv2.createBackgroundSubtractorMOG2()
                    #bk_frame = fgbg.apply(frame)
                        #center of camera (can be offset for manipulator)
                    frame = frame[0:int(mult_height*frame.shape[0]),0:int(mult_width*frame.shape[1])]
                    center_x = 0.95*frame.shape[1]+40
                    center_y = frame.shape[0]/2-20

                    [foundTarget,x,y,area,dir] = findTarget(frame, shape.color_lower, shape.color_upper)   # Send frame to target track function 
                    if foundTarget:
                        #cv2.drawContours(frame, [approx], -1, (0, 255, 0))  # Draw target polygon
                        cv2.circle(frame, (x,y), 2, (255,255,255), -1)      # Draw target center
                            #error from desired center
                        e_x = x-center_x
                        e_y = y-center_y

                        error = math.sqrt((e_x)**2 + (e_y)**2)
                        #print(error)
                      
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


def get_shape_information(input_image, min_area = 1000, max_area = 10000, color_bins = 5):
    blur = get_hypersmoothed_image(input_image, 3)
    blur_compact, c = do_kmeans_color_compaction(blur, color_bins)
    # Sorting by color intensity makes testing a bit more consistent
    c = sorted(c, key=lambda c: int(c[0])*int(c[0]) + int(c[1])*int(c[1]) + int(c[2])*int(c[2]))
    imret = blur_compact
    shape_list = list()
    center_x = 0.95*frame.shape[1]+40
                    center_y = frame.shape[0]/2-20
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
            if cl[0] < 30 and cl[2] > 20:   
                s.color = "red"
                s.box = 1
            elif cl[0] < 90 and cl[2] > 20:   
                s.color = "yellow"
                s.box = 2
            elif cl[0] < 150 and cl[2] > 20:  
                s.color = "green"
                s.box = 3
            elif cl[0] < 270 and cl[2] > 20:  
                s.color = "blue"
                s.box = 4
            elif cl[2] < 20:
                s.color = "black"
                s.box = 5
            else:
                print('no color detected')
            #s.color = cl
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
    high = np.array([hsv_col_high[0]+10, hsv_col_high[1]+20, val_high])
    low = np.array([hsv_col_low[0]-10,  hsv_col_low[1]-20,  val_low ])
    return [col_low, col_high, low, high]


def get_map():
    if cap.isOpened():
            rval, frame = cap.read()
            #fgbg = cv2.createBackgroundSubtractorMOG2()
            #bk_frame = fgbg.apply(frame)
            #vertices = np.array([[0,0],[0,frame.shape[0]],[frame.shape[1],0],[frame.shape[1],frame.shape[0]],[frame.shape[1],0.6*frame.shape[0]],[frame.shape[1],0.75*frame.shape[0]],[0.75*frame.shape[1],frame.shape[0]],[0.6*frame.shape[1],frame.shape[0]]], np.int32)
            #mask = np.zeros_like(img)
            # fill the mask
            #cv2.fillPoly(mask, vertices, 255)
            # now only show the area that is the mask
            #masked = cv2.bitwise_and(img, mask)
            frame = frame[0:int(mult_height*frame.shape[0]),0:int(mult_width*frame.shape[1])]
            shapes, annot_frame = get_shape_information(frame)

    return [shapes,annot_frame]


if __name__ == '__main__':
    try:
        print('Start Project 2 -- CV')
        cap = cv2.VideoCapture(1)
        #print('getting background')
        shapes, annot_frame = get_map()
        cv2.imwrite("map.png", annot_frame)
        print('map created')
        listener(shapes) #calls listener to find out what to do
        print('End Project 2 -- CV')
    except rospy.ROSInterruptException:
        pass