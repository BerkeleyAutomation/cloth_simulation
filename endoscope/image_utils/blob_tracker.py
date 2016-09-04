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
            # Stable Radix sort style ordering of blobs for consistency
            self.blobs.sort(key=lambda tup: tup[1])
            self.blobs.sort(key=lambda tup: tup[0])
        print self.blobs
        return self.blobs

def detect_and_dump_blobs(fname="images/blobs.p"):
    """
    Dumps a list of blobs in robot frame.
    """
    bt = BlobTracker()
    while True:
        blobs = bt.update_blobs()
        if blobs:
            with open(fname, "w+") as f:
                pickle.dump(blobs, f)
            break
        time.sleep(1)

def load_blobs(fname="images/blobs.p"):
    """
    Load a list of blobs in robot frame.
    """
    with open(fname, "rb") as f:
        try:
            return pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'

if __name__ == '__main__':
    bt = BlobTracker()
    for i in range(3):
        bt.update_blobs()
        time.sleep(2)

    detect_and_dump_blobs()