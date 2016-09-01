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


if __name__ == '__main__':
	experiment_directory = "experiment_data/experiments/2"
	policy_name = "policy.p"
	pts = "gauze_pts2.p"
	policy_file = osp.join(experiment_directory, policy_name)
	policy = load_policy(policy_file)
	pts = load_robot_points(osp.join(experiment_directory, pts))
	trajectory = np.array(get_robot_trajectory(pts))

	trajectory[:,2]+= 0.02

	grippers = GripperArm("PSM2", policy)
	scissors = ScissorArm("PSM1", trajectory, grippers)

	for i in range(len(trajectory)):
		scissors.step()

