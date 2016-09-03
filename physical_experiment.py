import numpy as np
import sys, os, pickle, time
from simulation import *
from tensioner import *
from shapecloth import *
from davinci_interface.interface import *
import IPython
import os.path as osp
from simulation_policy import *
from registration import *

"""
This file contains a script that takes a policy, a trajectory, and a pinning point, and executes it on the DVRK.
"""

if __name__ == '__main__':
	experiment_directory = "experiment_data/experiments/2"
	policy_name = "policy.p"
	pts = "gauze_pts2.p"
	policy_file = osp.join(experiment_directory, policy_name)
	pts_file = osp.join(experiment_directory, pts)
	corners_file = osp.join(experiment_directory, "gauze_pts.p")
	policy = load_policy(policy_file)
	pts = load_robot_points(osp.join(experiment_directory, pts))
	trajectory = np.array(get_robot_trajectory(pts))

	# trajectory = ? insert trajectory here

	# for safety/reproducibility
	trajectory[:,2]+= 0.02

	grippers = GripperArm("PSM2", policy)
	scissors = ScissorArm("PSM1", trajectory, grippers)

	# pt = [300, 300] pt in pixel space
	# grab_point = px_to_robot(pt, corners_file, pts_file)
	grap_pos = [-0.116102607139, 0.0817639759964, -0.0784721770163] # temporary grab point
	grippers.grab_point(pos)

	for i in range(len(trajectory)):
		scissors.step()

