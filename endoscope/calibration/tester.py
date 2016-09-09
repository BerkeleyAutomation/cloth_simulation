import sys, os, pickle, time
from rigid_transformation import *
import matplotlib.pyplot as plt
import numpy as np
from robot import *



if __name__ == '__main__':
  
	ORIENTATION = np.array([0.902174198372, 0.208355564829, -0.240495636706, -0.291258515964])
	HOME = np.array([0.0195990646306, 0.0274673676628, -0.0720500382531])


	left_pts = load_points("camera_data/left_chesspts").tolist()
	right_pts = load_points("camera_data/right_chesspts").tolist()

	pts3d = pixelto3d(left_pts, right_pts)

	psm1 = robot("PSM1")

	with open("camera_data/camera_psm1_rigid_transform.p", "rb") as f:
		transform = pickle.load(f)

	for pt in pts3d:
		rpt = np.ones((4, 1))
		rpt[:3, 0] = pt
		rpt = transform * rpt
		print rpt
		psm1.move_cartesian_frame_linear_interpolation(tfx.pose(rpt, ORIENTATION), 0.1)
		time.sleep(1)
		psm1.move_cartesian_frame_linear_interpolation(tfx.pose(HOME, ORIENTATION), 0.1)
		time.sleep(1)
