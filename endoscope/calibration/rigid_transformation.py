import image_geometry
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import cv2
import cv_bridge
import numpy as np
import rospy, scipy.misc
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import pickle
import sys
import time
from math import *

"""
This script contains utilities that are used to find the rigid transformation between coordinate frames.
"""

def load_points(fname="camera_data/psm1_calibration.p"):
    with open(fname, "rb") as f:
        try:
            return np.matrix(pickle.load(f))
        except EOFError:
            print 'Nothing written to file.'


def get_points_3d(left_points, right_points, info):
    """ this method assumes that corresponding points are in the right order
        and returns a list of 3d points """

    # both lists must be of the same lenghth otherwise return None
    if len(left_points) != len(right_points):
        rospy.logerror("The number of left points and the number of right points is not the same")
        return None

    points_3d = []
    for i in range(len(left_points)):
        a = left_points[i]
        b = right_points[i]
        disparity = sqrt((a[0]-b[0]) ** 2 + (a[1] - b[1]) ** 2)
        pt = convertStereo(a[0], a[1], disparity, info)
        points_3d.append(pt)
    return points_3d


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

def solve_for_camera_matrix(rpts="camera_data/psm1_calibration.p", cpts="camera_data/endoscope_chesspts.p"):
    """
    Returns Camera -> Robot frame matrix
    """
    robot_points = load_points(rpts)
    camera_points = load_points(cpts)
    camera_mean = camera_points.mean(axis=0)
    robot_mean = robot_points.mean(axis=0)
    for i in range(robot_points.shape[0]):
        robot_points[i,:] -= robot_mean
        camera_points[i,:] -= camera_mean
    X = camera_points.T
    Y = robot_points.T
    covariance = X * Y.T
    U, Sigma, V = np.linalg.svd(covariance)
    V = V.T
    idmatrix = np.identity(3)
    idmatrix[2, 2] = np.linalg.det(V * U.T)
    R = V * idmatrix * U.T
    t = robot_mean.T - R * camera_mean.T
    return np.concatenate((R, t), axis=1)

def solve_for_rigid_transformation(inpts, outpts):
    """
    Takes in two sets of corresponding points, returns the rigid transformation matrix from the first to the second.
    """

    inpt_mean = inpts.mean(axis=0)
    outpt_mean = outpts.mean(axis=0)
    for i in range(outpts.shape[0]):
        outpts[i,:] -= outpt_mean
        inpts[i,:] -= inpt_mean
    X = inpts.T
    Y = outpts.T
    covariance = X * Y.T
    U, Sigma, V = np.linalg.svd(covariance)
    V = V.T
    idmatrix = np.identity(3)
    idmatrix[2, 2] = np.linalg.det(V * U.T)
    R = V * idmatrix * U.T
    t = outpt_mean.T - R * inpt_mean.T
    return np.concatenate((R, t), axis=1)


def write_mat_to_file(filename, matrix):
    with open("camera_data/" + filename, "w+") as f:
        pickle.dump(matrix, f)

def pixelto3d(left, right):
    info = {}
    with open("camera_data/camera_info.p") as f:
        info = pickle.load(f)
    pts3d = get_points_3d(left, right, info)
    return np.matrix([(p.point.x, p.point.y, p.point.z) for p in pts3d])

def least_squares_plane_normal(points_3d):
    x_list = points_3d[:,0]
    y_list = points_3d[:,1]
    z_list = points_3d[:,2]

    A = np.concatenate((x_list, y_list, np.ones((len(x_list), 1))), axis=1)
    plane = np.matrix(np.linalg.lstsq(A, z_list)[0]).T

    return plane

def distance_to_plane(m, point):
    A = m[0,0]
    B = m[0,1]
    C = -1
    D = m[0,2]
    p0 = np.array([0,0,D])
    p1 = np.array(point)
    n = np.array([A,B,C])/np.linalg.norm(np.array([A,B,C]))
    return np.dot(np.absolute(p0 - p1),n)

def get_good_indices(thresh=0.022):
    camera_points = load_camera_points()
    plane = least_squares_plane_normal(camera_points)
    good_pts = []
    for i in range(camera_points.shape[0]):
        p = camera_points[i,:]
        dist = distance_to_plane(plane, p)
        if abs(dist) > thresh:
            continue
        else:
            good_pts.append(i)
    return good_pts

def plot_camera_points(camera_points):
    """
    Plots points in camera_frame. Axes may need to be edited.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(camera_points[:,0]), np.array(camera_points[:,1]), np.array(camera_points[:,2]),c='r')

    ax.set_xlim3d(-0.05, 0.05)
    ax.set_ylim3d(-0.05, 0.05)
    ax.set_zlim3d(0.1, 0.2)
    plt.show()

def plot_training_error(R, t):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    cmat = solve_for_camera_matrix()
    camera_points = load_camera_points()
    robot_points = load_robot_points()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(robot_points[:,0]), np.array(robot_points[:,1]), np.array(robot_points[:,2]),c='r')

    ax.scatter(np.array(rnew[:,0]), np.array(rnew[:,1]), np.array(rnew[:,2]))
    plt.show()

def skew(vector):
    return np.matrix([[0, -vector[2], vector[1]], 
                     [vector[2], 0, -vector[0]], 
                     [-vector[1], vector[0], 0]])

if __name__ == '__main__':

    cmat = solve_for_camera_matrix()
    write_mat_to_file("camera_psm1_rigid_transform.p", cmat)
    print cmat

    rpts="camera_data/psm1_calibration.p"
    cpts="camera_data/endoscope_chesspts.p"
    camera_points = load_points(cpts)
    robot_points = load_points(rpts)
    # plot_camera_points(camera_points)


    left_pts = load_points("camera_data/left_chesspts").tolist()
    right_pts = load_points("camera_data/right_chesspts").tolist()

    print right_pts
    print left_pts

    img = np.array(cv2.imread("camera_data/right_checkerboard.jpg"))
    print img.shape
    # plt.imshow(img)
    # plt.show()

    plt.scatter(np.array(right_pts)[:,0], np.array(right_pts)[:,1])
    plt.show()
    plt.scatter(np.array(left_pts)[:,0], np.array(left_pts)[:,1])
    plt.show()
