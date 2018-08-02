#!/usr/bin/env python
import cv2
import numpy as np

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


# Machine Vision Shape
# Struct with useful shape information, especially color and x,y location.
#
# The contour is a cv2 contour, and can be used for further post-processing if
# desired.
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


# Retrieves shape information from the given image. Filters out shapes with areas over
# max_area and under min_area. Uses color_bins number of color bins in the k-means
# calculator.
#
# Algorithm was developed with simple, colorful geometry on plain backgrounds, and the
# returned shape information could still benefit from further filtering. If the algorithm
# is used on other things, it may yield unpredictable results.
#
# Test on a sample environment and tune before trusting to an automated system. 
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


def minimal_run():
	cap = cv2.CaptureFromCAM(0)
	while 1:
		if cap.isOpened():
			rval, frame = cap.read()
			cv2.imshow("Raw Image", frame)
			shapes, annot_frame = get_shape_information(frame)
			for s in shapes:
				print(s.color)
			cv2.imshow("Processed Image", annot_frame)
			k = cv2.waitKey(5) & 0xFF
			if k == 27: # ESC
				break
	cv2.destroyAllWindows()
	cap.release()


if __name__ == "__main__":
	minimal_run()
