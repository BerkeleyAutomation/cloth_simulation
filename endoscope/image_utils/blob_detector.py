# import the necessary packages
import argparse
import imutils
import cv2
import numpy as np
import matplotlib.pyplot as plt
import IPython
import pickle
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import sys
import time
import image_geometry
import scipy.interpolate
from sklearn.neighbors import KNeighborsClassifier


### TO DO: right image/left image preprocessing to increase robustness

def contour_detector(image, show_plots = False, rescale=2):
    # resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=int(np.ceil(image.shape[1]/rescale)))
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (25, 25), 0)
    # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 19, 2)
    
    # cv2.dilate(thresh, thresh, iterations=1)
    kernel = np.ones((10, 10), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    thresh = 255 - closing
    # edges = cv2.Canny(blurred, 100, 200)
    if show_plots:
        cv2.imshow("Thresh", thresh)
        cv2.waitKey(0)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    # find contours in the thresholded image 
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # initialize the shape detector
    # sd = ShapeDetector()
    lst = []
    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        cY = int((M["m01"] / M["m00"]) * ratio) if M["m00"] != 0 else 0
        # shape = sd.detect(c)
        #find mean color

        mask = np.zeros(gray.shape,np.uint8)
        cv2.drawContours(mask,[c],0,255,-1)
        mean_val = cv2.mean(hsv,mask = mask)
        if cv2.contourArea(c) > 2000 or cv2.contourArea(c) < 500 or mean_val[0] < 90 or mean_val[2] < 70:
            continue
        lst.append([mean_val[:3], (cX, cY), cv2.contourArea(c)])

        # print [mean_val[:3], (cX, cY)]
        if show_plots:
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, "x", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
              0.5, (255, 255, 255), 2)

            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)

    # cv2.imshow('image', image)
    # cv2.waitKey(0)
    return lst

def find_correspondences(left, right, disparity_max, disparity_min=0, blob_area_disparity=200, blob_max_area=2000, debugging=False):
    indices = []
    correspondences = []
    for i in range(len(left)):
        # blob_left = [(h,s,v), (cX, cY)], a list of hsv mean value and center of blob pairs in original image
        blob_left = left[i]
        mean_left = blob_left[0]
        center_left = np.array(blob_left[1])

        best_idx = -1
        best_dist = float("inf")
        for j in range(len(right)):
            # check if the current blob in the right list is closer than our threshold radius
            blob_right = right[j]
            mean_right = blob_right[0]
            center_right = np.array(blob_right[1])

            dist = np.linalg.norm(center_left - center_right)
            if dist < disparity_max and dist < best_dist and dist >= disparity_min and blob_left[2] < blob_max_area:
                # check the h value of the means to see if they are with +-10 of each other
                if abs(mean_left[0] - mean_right[0]) < 30 and center_left[0] > center_right[0]:
                    if abs(center_right[1] - center_left[1]) < 20 and abs(blob_left[2] - blob_right[2]) < blob_area_disparity:
                        best_dist = dist
                        best_idx = j
        indices.append(best_idx)
        if best_idx != -1:
            correspondences.append((tuple(center_left.tolist()), right[best_idx][1]))

    return correspondences

def calculate_disparity(correspondences):
    lst = []
    for c in correspondences:
        left = c[0]
        right = c[1]
        disparity = left[0] - right[0]
        lst.append(disparity)
    return lst

def convertStereo(u, v, disparity, info):
    """
    Converts two pixel coordinates u and v along with the disparity to give PointStamped       
    """
    stereoModel = image_geometry.StereoCameraModel()
    stereoModel.fromCameraInfo(info['l'], info['r'])
    (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)

    cameraPoint = PointStamped()
    cameraPoint.header.frame_id = info['l'].header.frame_id
    cameraPoint.header.stamp = time.time()
    cameraPoint.point = Point(x,y,z)
    return cameraPoint

def get_points_3d(info, left_points, right_points):
    """ this method assumes that corresponding points are in the right order
        and returns a list of 3d points """

    # both lists must be of the same lenghth otherwise return None
    if len(left_points) != len(right_points):
        return None

    points_3d = []
    for i in range(len(left_points)):
        a = left_points[i]
        b = right_points[i]
        disparity = abs(a[0]-b[0])
        pt = convertStereo(a[0], a[1], disparity, info)
        points_3d.append([pt.point.x, pt.point.y, pt.point.z])
    return np.array(points_3d)

def fit_surface(pts3d):
    pts3d = np.array(pts3d)
    x = pts3d[:,0]
    y = pts3d[:,1]
    z = pts3d[:,2]
    return scipy.interpolate.interp2d(x, y, z, kind='linear')

def leftpixels_to_cframe(surf, left_pts, right_pts, pts3d, x, y, knn=False):
    xin = np.array([a[0] for a in left_pts])
    bias = np.ones(len(xin))
    yin = np.array([a[1] for a in left_pts])
    xout = np.array([np.ravel(a)[0] for a in pts3d])
    yout = np.array([np.ravel(a)[1] for a in pts3d])
    A = np.vstack([xin, bias]).T
    m1, c1 = np.linalg.lstsq(A, xout)[0]

    A = np.vstack([yin, bias]).T
    m2, c2 = np.linalg.lstsq(A, yout)[0]

    xnew = m1 * x + c1
    ynew = m2 * y + c2
    zold = surf.f2(xnew, ynew)[0]

    cpoint = np.matrix([(xnew, ynew, zold)])
    pt = np.ones(4)
    pt[:3] = cpoint
    pred = np.ravel(surf.cmat * np.matrix(pt).T)
    if knn:
        return (pred[0], pred[1], surf.query_knn(pred[0], pred[1])[2])
    return (pred[0], pred[1], surf.query(pred[0], pred[1])[2])

def get_blobs(left, right, fname=False):
    with open("../calibration/camera_data/camera_info.p", "rb") as f:
        info = pickle.load(f)

    if fname:
        left_image = cv2.imread(left)
        right_image = cv2.imread(right)
    else:
        left_image, right_image = left, right
    left = contour_detector(left_image)
    right = contour_detector(right_image)
    correspondences = find_correspondences(left, right, 300, 70)
    print "found ", len(correspondences), " correspondences"
    left_pts = [a[0] for a in correspondences]
    right_pts = [a[1] for a in correspondences]
    pts3d = get_points_3d(info, left_pts, right_pts)
    oldpts3d = pts3d
    newpts = []
    with open("../calibration/camera_data/camera_psm1_rigid_transform.p", "rb") as f:
        cmat = pickle.load(f)
    for cpoint in pts3d:
        pt = np.ones(4)
        pt[:3] = cpoint
        pred = cmat * np.matrix(pt).T
        newpts.append(np.ravel(pred).tolist())
    pts3d = newpts
    return pts3d
    




if __name__ == "__main__":

    SHOW_PLOTS = True
    
    with open("../calibration/camera_data/camera_info.p", "rb") as f:
        info = pickle.load(f)

    surf = get_blobs("images/left.jpg", "images/right.jpg", True) # specify images

    print surf