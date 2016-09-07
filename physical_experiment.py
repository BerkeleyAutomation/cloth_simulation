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
from notch_finder import *
from endoscope.image_utils.blob_tracker import *

"""
This file contains a script that takes a policy, a trajectory, and a pinning point, and executes it on the DVRK.
"""

if __name__ == '__main__':
    experiment_directory = "experiment_data/experiments/4"
    policy_name = "three.p"
    pts = "gauze_pts2.p"
    policy_file = osp.join(experiment_directory, policy_name)
    pts_file = osp.join(experiment_directory, pts)
    corners_file = osp.join(experiment_directory, "gauze_pts.p")
    policy = load_policy(policy_file)
    pts = load_robot_points(osp.join(experiment_directory, pts))
    trajectory = np.array(get_robot_trajectory(pts))
    trajlen = len(trajectory)

    with open(osp.join(experiment_directory, "trajindices.p"), "rb") as f:
        indices = pickle.load(f)
    lst = []
    for elem in indices:
        lst.append(trajectory[elem].tolist())
    trajectory = lst
    with open(osp.join(experiment_directory, "blobs.p"), "rb") as f:
        blobs = pickle.load(f)
    bt = BlobTracker(blobs)
    time.sleep(1)
    for i in range(10):
        bt.update_blobs()

    # for safety/reproducibility
    # trajectory[:,2]+= 0.02

    grippers = GripperArm("PSM2", policy)
    scissors = ScissorArm("PSM1", trajectory, grippers)

    pt = [280, 480]
    grab_point = px_to_robot(pt, corners_file, pts_file)
    grab_point = [-0.112095781205, 0.0755202298493, -0.0785452182936]
    # grippers.grab_point(grab_point)

    for i in range(trajlen):
        blobs = bt.update_blobs()
        simblobs = []
        for blob in blobs:
            simblob = robot_frame_to_sim_file(blob, corners_file, pts_file, offset=(50,50))
            simblobs.append(simblob[0])
            simblobs.append(simblob[1])
            simblobs.append(simblob[2])
        scissors.step(simblobs)

