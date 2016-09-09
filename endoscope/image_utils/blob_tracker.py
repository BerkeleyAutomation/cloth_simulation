import numpy as np
import sys, os, time, pickle
import matplotlib.pyplot as plt
from blob_detector import *
sys.path.append(os.path.dirname(os.getcwd()))
from robot import *
from image_subscriber import *
import scipy
from sklearn.neighbors import KNeighborsRegressor
import IPython

class BlobTracker(object):

    def __init__(self, blobs=None):
        self.image_subscriber = ImageSubscriber()
        if blobs:
            self.blobs = [np.array(blob) for blob in blobs]
        else:
            self.blobs = blobs
        self.threshold= 0.01

    @property
    def left_image(self):
        return self.image_subscriber.left_image

    @property
    def right_image(self):
        return self.image_subscriber.right_image    
    
    def update_blobs(self):
        if self.left_image != None and self.right_image != None:
            newblobs = [np.array(blob) for blob in get_blobs(self.left_image, self.right_image)]
            if self.blobs:
                lst = []
                for blob in self.blobs:
                    dists = []
                    for b in newblobs:
                        dists.append(np.linalg.norm(blob - b))
                    closest, bestdist = np.argmin(dists), np.min(dists)
                    if bestdist < self.threshold:
                        lst.append([closest, bestdist])
                    else:
                        lst.append([-1, -1])
                tmp = [[0] for i in range(len(self.blobs))]
                for i in range(len(tmp)):
                    if lst[i][0] >= 0:
                        tmp[i] = newblobs[lst[i][0]]
                good_blobs = [a for a in tmp if len(a) != 1]
                for i in range(len(tmp)):
                    if len(tmp[i]) == 1:
                        self.interpolate_blobs2(self.blobs, tmp)
                        # tmp[i] = self.blobs[i]
                self.blobs = tmp
            else:
                self.blobs = newblobs
        print self.blobs
        return self.blobs

    def interpolate_blobs(self, good_blobs, missing_blob):
        X = np.vstack(good_blobs)[:,:2]
        y = np.vstack(good_blobs)[:,2]
        h = neigh = KNeighborsRegressor(n_neighbors=2)
        h = neigh.fit(X, y)
        x1 = h.predict([missing_blob.tolist()[:2]])
        X = np.vstack(good_blobs)[:,(0,2)]
        y = np.vstack(good_blobs)[:,1]
        h = neigh = KNeighborsRegressor(n_neighbors=2)
        h = neigh.fit(X, y)
        x2 = h.predict([[missing_blob[0], missing_blob[2]]])
        X = np.vstack(good_blobs)[:,1:]
        y = np.vstack(good_blobs)[:,0]
        h = neigh = KNeighborsRegressor(n_neighbors=2)
        h = neigh.fit(X, y)
        x3 = h.predict([missing_blob.tolist()[1:]])
        return np.ravel(np.array([x3, x2, x1]))

    def interpolate_blobs2(self, old_blobs, new_blobs):
        lst1 = []
        lst2 = []
        for i in range(len(old_blobs)):
            oblob = old_blobs[i]
            nblob = new_blobs[i]
            if len(nblob) == 3:
                lst1.append(list(oblob))
                lst2.append(list(nblob))
        for i in range(len(new_blobs)):
            if len(new_blobs[i]) == 1:
                dists = np.vstack(lst1) - np.tile(np.array(old_blobs[i]), (len(lst1), 1))
                dists = np.linalg.norm(dists, axis=1)
                closest = np.argmin(dists)
                delta = np.array(lst2[closest]) - np.array(lst1[closest])
                new_blobs[i] = old_blobs[i] + delta

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
    time.sleep(1)
    for i in range(60):
        bt.update_blobs()

    # detect_and_dump_blobs()
