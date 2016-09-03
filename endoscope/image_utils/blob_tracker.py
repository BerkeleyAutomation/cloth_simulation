import numpy as np
import sys, os, time, pickle
import matplotlib.pyplot as plt
from blob_detector import *
sys.path.append(os.path.dirname(os.getcwd()))
from calibration.robot import *
from image_subscriber import *

class BlobTracker(object):

	def __init__(self):
		self.image_subscriber = ImageSubscriber()
		self.blobs = None

	@property
	def left_image(self):
		return self.image_subscriber.left_image

	@property
	def right_image(self):
		return self.image_subscriber.right_image	
	
	def update_blobs(self):
		if self.left_image != None and self.right_image != None:
			self.blobs = get_blobs(self.left_image, self.right_image)
		print self.blobs

if __name__ == '__main__':
	bt = BlobTracker()
	for i in range(10):
		bt.update_blobs()
		time.sleep(2)