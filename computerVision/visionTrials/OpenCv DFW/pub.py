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

cap = cv2.VideoCapture(1)


# Return an image with colors replaced by the K most common colors
# Now it also returns the colors it found / used as the second element
#
# Simplifies color space, but both slow and not likely to be that useful
# --- scratch that, works pretty good after a hypersmooth
#
# Note: there's a few optimizations that could be made here, but in the
# interests of not getting bogged down fixing what works, I'll let it slide
def do_kmeans_color_compaction(image, K):
    Z = image.reshape((-1, 3))
    Z = np.float32(Z)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 2.5)
    compactness, labels, centers = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    centers = np.uint8(centers)
    res = centers[labels.flatten()]
    res2 = res.reshape((image.shape))
    return [res2, centers.reshape((-1, 3))]


# Base blob detector, for use in do_simple_blob_detect*() functions
def _bd_kp(img):
    blob_detector = cv2.SimpleBlobDetector_create()
    blobs_maybe = blob_detector.detect(img)
    return blobs_maybe


# Return image with detected blobs circled
def do_simple_blob_detect(image):
    blobs_maybe = _bd_kp(image)
    blur_mit_kp = cv2.drawKeypoints(image, blobs_maybe, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return blur_mit_kp
    

# Return only centerpoints of blobs
def do_simple_blob_detect_kp_only():
    blobs_maybe = _bd_kp(image)
    ret = [x.pt for x in blobs_maybe]
    return ret


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
    return [col_low, col_high]


def cycle_erode_dilate(img, size, iters):
    kernel = np.ones((size, size),np.uint8)
    erosion = cv2.erode(img,kernel,iterations = iters)
    dilation = cv2.dilate(erosion,kernel,iterations = iters)
    return dilation


class MvShape:
    def __init__(self):
        self.contour = None
        self.raw_contour = None
        self.cg = None
        self.color = None
        self.area = None
        self.cg_from_center = None

    def __str__(self):
        return str(self.cg) + ", " + str(self.color) + ", " + str(self.area)


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
        c_low, c_up = get_color_bounds_for_threshold(blur_compact, cl, 8)
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
            shape_list.append(s)

            imret = cv2.drawContours(imret, [s.contour], -1, (0,255,0), 1, maxLevel = 1)
            imret = cv2.circle(imret, (cx, cy), 2, (255, 0, 0), -1)
            imret = cv2.putText(imret, str(s.cg_from_center), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
    return [shape_list, imret]


def findTarget(image, lower, upper):
    shape_list, rframe = get_shape_information(image)
    #print rframe.shape
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

                return[True, x, y, area, approx, dir, rframe]
            break
    return[False, 0, 0, 0, 0, 0, rframe]

def talker():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
    while 1:
        if cap.isOpened(): # try to get the first frame
            #black, blue, red, yellow, green

            rval, frame = cap.read()

            [foundTarget,x,y,area,approx,dirn,filterframe] = findTarget(frame, lower_blue, upper_blue)   # Send frame to target track function 
            if foundTarget:
                cv2.drawContours(frame, [approx], -1, (0, 255, 0))  # Draw target polygon
                cv2.circle(frame, (x,y), 2, (255,255,255), -1)      # Draw target center

                #a = [x,y]
                #pub_a = Int32MultiArrary(data=a)
                pub_a = "(" + str(x) + ", " + str(y) + ")"

                rospy.loginfo(pub_a)
#                pub.publish(pub_a)
#                rate.sleep()
            else:

                #a = [-9999, rospy.get_time()]
                #pub_a = Int32MultiArrary(data=a)
                pub_a = "Target not found. "

                rospy.loginfo(pub_a)
#                pub.publish(pub_a)
#                rate.sleep()

        else:

            #a = [0 , rospy.get_time()]
            #pub_a = Int32MultiArrary(data=a)
            pub_a = "Video not opened. "

            rospy.loginfo(pub_a)
#            pub.publish(pub_a)
#            rate.sleep()
        
        cv2.imshow("Image",frame)
        cv2.imshow("FILTER",filterframe)
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