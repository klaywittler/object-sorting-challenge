import cv2
import numpy as np	
import math

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