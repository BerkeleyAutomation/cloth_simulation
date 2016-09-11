import numpy as np
import sys, os, pickle, copy
import IPython
import argparse
import imutils
import cv2
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import time
import image_geometry
from blob_detector import *
from image_subscriber import *
from scipy.stats import threshold

class LineDetector(object):

	def __init__(self):
		self.image_subscriber = ImageSubscriber()


	def get_pixel_from3D(self, camera_matrix, position, camera_info, offset=(0,0)):
	    Trobot = np.zeros((4,4))
	    Trobot[:3,:] = np.copy(camera_matrix)
	    Trobot[3,3] = 1
	    Rrobot = np.linalg.inv(Trobot)

	    x = np.ones((4,1))

	    x[:3,0] = np.squeeze(position)

	    cam_frame = np.dot(Rrobot,x)

	    Pcam = np.array(camera_info.P).reshape(3,4)

	    V = np.dot(Pcam, cam_frame)

	    V = np.array((int(V[0]/V[2]), int(V[1]/V[2])))

	    V[0] = V[0] + offset[0]
	    V[1] = V[1] + offset[1]

	    return V

	def line_detector_drawn(self, image, show_plots = False):
	    # resize it to a smaller factor so that
	    # the shapes can be approximated better
	    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/2)))
	    ratio = image.shape[0] / float(resized.shape[0])

	    # convert the resized image to grayscale, blur it slightly,
	    # and threshold it
	    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
	    blurred = cv2.GaussianBlur(gray, (45, 45), 0)

	    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
	    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
	        cv2.THRESH_BINARY, 19, 2)
	    kernel1 = np.ones((7, 7), np.uint8)
	    thresh = 255 - thresh

	    thresh = cv2.dilate(thresh, kernel1, iterations=1)
	    kernel2 = np.ones((7, 7), np.uint8)
	    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel2)
	    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel2)
	    thresh = closing
	    # if show_plots:
	    #     cv2.imshow("Thresh", thresh)
	    #     cv2.waitKey(0)

	    return self.remove_blobs(image, resized, thresh, ratio, show_plots), ratio


	def remove_blobs(self, full_image, resized_image, gray, ratio, show_plots=False):
	    if show_plots:
	        cv2.imshow("Thresh2", gray)
	        cv2.waitKey(0)

	    cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,
	        cv2.CHAIN_APPROX_SIMPLE)
	    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

	    # loop over the contours
	    
	    hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
	    minv = 1000
	    for c in cnts:
	        mask = np.zeros(gray.shape,np.uint8)
	        cv2.drawContours(mask,[c],0,255,-1)
	        mean_val = np.array(cv2.mean(hsv,mask = mask))
	        minv = min(mean_val[2], minv)

	    for c in cnts:
	        mask = np.zeros(gray.shape,np.uint8)
	        cv2.drawContours(mask,[c],0,255,-1)
	        mean_val = np.array(cv2.mean(hsv,mask = mask))
	        if np.max(mean_val) < 100 or mean_val[2] < minv:
	            continue
	        else:
	            if cv2.contourArea(c) < 2000:
	                cv2.drawContours(gray, [c], -1, (0, 0, 0), -1)
	            else:
	                pass

	    if show_plots:
	        cv2.imshow("a", gray)
	        cv2.waitKey(0)
	    return gray

	def onLine(self, image, pt):
		new, ratio = self.line_detector_drawn(image)
		if new[pt[1], pt[0]] != 0:
			return True
		return False

	def query(self, pt):
		return ld.onLine(self.image_subscriber.left_image, pt)

if __name__ == '__main__':
	img = cv2.imread("images/left.jpg")
	ld = LineDetector()
	new, ratio = ld.line_detector_drawn(img)
	plt.imshow(new, cmap='Greys')
	plt.show()
	print ld.onLine(img, (489,327))
