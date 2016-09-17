import cv2
import rospy, pickle, time
from robot import *
from geometry_msgs.msg import Pose
import numpy as np
import PyKDL
import multiprocessing
import tfx
import fitplane
from scipy.interpolate import interp1d
from shape_tracer import plot_points
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import notch
from geometry_msgs.msg import Point
from image_saver import ImageSaver
import least_square_circle as sqcirc
from mpl_toolkits.mplot3d import Axes3D
from ImageSubscriber import ImageSubscriber
from math import *
from scipy.spatial import ConvexHull
	
def process_img(fname):
	""" converts image to a binary img and thins a little"""
	img = cv2.imread(fname,1)
	resized=cv2.resize(img,None,fx=.5, fy=.5, interpolation = cv2.INTER_CUBIC)

	print resized.shape
	gray=cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
	ret,thresh2 = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
	kernel = np.ones((8,8),np.uint8)
	closing = cv2.morphologyEx(thresh2, cv2.MORPH_CLOSE, kernel)
	erosion = cv2.erode(closing,kernel,iterations = 1)
	final = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, kernel)
	return final




def get_raw_points(img):
	"""finds coordinates of all lit up spots"""
	pts=[]
	x=[]
	y=[]
	for i in range(img.shape[0]):
		for j in range(img.shape[1]):
			if img[i][j]==255:
				pts.append([i,j])
				x.append(i)
				y.append(j)
	return pts,x,y

		




def neighbours(x,y,image):
    """Return 8-neighbours of image point P1(x,y), in a clockwise order"""
    img = image
    x_1, y_1, x1, y1 = x-1, y-1, x+1, y+1
    return [ img[x_1][y], img[x_1][y1], img[x][y1], img[x1][y1],     # P2,P3,P4,P5
                img[x1][y], img[x1][y_1], img[x][y_1], img[x_1][y_1] ]    # P6,P7,P8,P9

def transitions(neighbours):
    """No. of 0,1 patterns (transitions from 0 to 1) in the ordered sequence"""
    n = neighbours + neighbours[0:1]      # P2, P3, ... , P8, P9, P2
    return sum( (n1, n2) == (0, 1) for n1, n2 in zip(n, n[1:]) )  # (P2,P3), (P3,P4), ... , (P8,P9), (P9,P2)


def zhangSuen(image):
    """the Zhang-Suen Thinning Algorithm"""
    Image_Thinned = image.copy()  # deepcopy to protect the original image
    changing1 = changing2 = 1        #  the points to be removed (set as 0)
    while changing1 or changing2:   #  iterates until no further changes occur in the image
        # Step 1
        changing1 = []
        rows, columns = Image_Thinned.shape               # x for rows, y for columns
        for x in range(1, rows - 1):                     # No. of  rows
            for y in range(1, columns - 1):            # No. of columns
                P2,P3,P4,P5,P6,P7,P8,P9 = n = neighbours(x, y, Image_Thinned)
                if (Image_Thinned[x][y] == 1     and    # Condition 0: Point P1 in the object regions 
                    2 <= sum(n) <= 6   and    # Condition 1: 2<= N(P1) <= 6
                    transitions(n) == 1 and    # Condition 2: S(P1)=1  
                    P2 * P4 * P6 == 0  and    # Condition 3   
                    P4 * P6 * P8 == 0):         # Condition 4
                    changing1.append((x,y))
        for x, y in changing1: 
            Image_Thinned[x][y] = 0
        # Step 2
        changing2 = []
        for x in range(1, rows - 1):
            for y in range(1, columns - 1):
                P2,P3,P4,P5,P6,P7,P8,P9 = n = neighbours(x, y, Image_Thinned)
                if (Image_Thinned[x][y] == 1   and        # Condition 0
                    2 <= sum(n) <= 6  and       # Condition 1
                    transitions(n) == 1 and      # Condition 2
                    P2 * P4 * P8 == 0 and       # Condition 3
                    P2 * P6 * P8 == 0):            # Condition 4
                    changing2.append((x,y))    
        for x, y in changing2: 
            Image_Thinned[x][y] = 0
    return Image_Thinned

def fit_plane(x,y,scale=.5,z=69):#scale and z needs to be experimentally gotten
	"""fits to the frame of the robot and the plane of the gauss"""
	traj=[]
	for i in range(len(x)):
		traj.append([x[i],y[i],z])
	return traj,x,y,z

def plot_points(x,y,z,centroid):
	
	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d') 
	# ax.scatter(x,y,z)
	plt.plot(x,y)
	plt.scatter(centroid[0],centroid[1],color='r')
	plt.show()

def centroid(pts):
	# pts=np.array(pts)
	return np.array([(sum(pts[:,0])/len(pts[:,0])),(sum(pts[:,1])/len(pts[:,1]))])
def angle_calc(pt1,pt2):
	x1,y1=pt1[0],pt1[1]
	x2,y2=pt2[0],pt2[1]
	return atan2(x2-x1,y2-y1)

# def reorganize(pts,centroid):
# 	angle=0
# 	count=0
# 	new_pts=[]
# 	print pts
# 	while angle<6.3:
# 		for i in range(len(pts)):
# 			measured=round(angle_calc(centroid,pts[i]),3)
			
# 			if measured==angle:
# 				#print measured
# 				new_pts.append([pts[i,0],pts[i,1]])
# 		angle+=.005
		

# 	return np.array(new_pts)


if __name__ == '__main__':
		# a=ImageSubscriber()
	
	processed=process_img('image_utils/left5.jpg')
	# processed=process_img('image_utils/right1.jpg')
	
	# edges = cv2.Canny(processed,0,255)
	thinned=zhangSuen(processed)
	cv2.imshow('processed',thinned)


	print processed.shape
	cv2.waitKey(5)
	cv2.destroyAllWindows()
	pts,x,y =get_raw_points(thinned)
	pts=np.array(pts)
	centroid=centroid(pts)
	print centroid

	pts=pts[ConvexHull(pts).vertices]
	traj,x,y,z =fit_plane(x,y)
	# print pts
	

	plot_points(pts[:,0],pts[:,1],z,centroid)
	
