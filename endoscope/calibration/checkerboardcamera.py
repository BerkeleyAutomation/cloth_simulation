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
import pickle
import sys


USE_SAVED_IMAGES = False

"""
This script detects chessboard corners from the endoscope cameras. It writes the points to calibration_data/endoscope_points.
"""


def convertStereo(u, v, disparity, info):
    """
    Converts two pixel coordinates u and v along with the disparity to give PointStamped       
    """
    stereoModel = image_geometry.StereoCameraModel()
    stereoModel.fromCameraInfo(info['l'], info['r'])
    (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)

    cameraPoint = PointStamped()
    cameraPoint.header.frame_id = info['l'].header.frame_id
    cameraPoint.header.stamp = rospy.Time.now()
    cameraPoint.point = Point(x,y,z)
    return cameraPoint

class ChessDetector:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image = None
        self.right_image = None
        self.info = {'l': None, 'r': None, 'b': None, 'd': None}
        self.plane = None

        #========SUBSCRIBERS========#
        # image subscribers
        rospy.Subscriber("/endoscope/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/endoscope/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/endoscope/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/endoscope/right/camera_info",
                         CameraInfo, self.right_info_callback)

    def left_info_callback(self, msg):
        if self.info['l']:
            return
        self.info['l'] = msg

    def right_info_callback(self, msg):
        if self.info['r']:
            return
        self.info['r'] = msg

    def right_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        if USE_SAVED_IMAGES:
            self.right_image = cv2.imread('right_checkerboard.jpg')
        else:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            scipy.misc.imsave('camera_data/right_checkerboard.jpg', self.right_image)


    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        if USE_SAVED_IMAGES:
            self.left_image = cv2.imread('left_checkerboard.jpg')
        else:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            scipy.misc.imsave('camera_data/left_checkerboard.jpg', self.left_image)
        if self.right_image != None:
            self.process_image()
    
    def get_points_3d(self, left_points, right_points):
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
            disparity = abs(a[0]-b[0])
            pt = convertStereo(a[0], a[1], disparity, self.info)
            points_3d.append(pt)
        return points_3d

    def process_image(self):
        print "processing image"
        # # Code for 2D pixels in rectified image -> 3D point in Camera Frame. Uncomment to use
        # left_corners = [(997, 893)]
        # right_corners = [(888, 898)]
        # self.pts = [(p.point.x, p.point.y, p.point.z) for p in pts3d]
        # print self.pts
        # return


        left_gray = cv2.cvtColor(self.left_image,cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.right_image,cv2.COLOR_BGR2GRAY)
        x, y = 5, 5
        ret, left_corners = cv2.findChessboardCorners(left_gray, (x,y), flags=1)
        ret, right_corners = cv2.findChessboardCorners(right_gray, (x,y), flags=1)
        print left_corners
        print right_corners
        print len(left_corners), len(right_corners)
        left, right, = [], []
        for i in range(len(left_corners)):
            left.append([left_corners[i][0][0], left_corners[i][0][1]])
            right.append([right_corners[i][0][0], right_corners[i][0][1]])
        pts3d = self.get_points_3d(left, right)
        self.pts = [(p.point.x, p.point.y, p.point.z) for p in pts3d]
        if len(left_corners) == len(right_corners) == x*y:
            with open('camera_data/endoscope_chesspts.p', "w+") as f:
                pickle.dump(self.pts, f)
            with open('camera_data/camera_info.p', 'w+') as f:
                pickle.dump(self.info, f)
            with open('camera_data/left_chesspts', 'w+') as f:
                pickle.dump(left_corners, f)
            with open('camera_data/right_chesspts', 'w+') as f:
                pickle.dump(right_corners, f)
            rospy.signal_shutdown("Finished.")



if __name__ == "__main__":
    rospy.init_node('circle_detector')
    a = ChessDetector()
    rospy.spin()
