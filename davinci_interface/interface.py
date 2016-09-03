from robot import *
import sys, os, pickle, copy, time
import numpy as np
import tfx
import os.path as osp
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

MAPPING = {
    0 : (0,0,0),
    1 : (-1,0,0),
    2 : (0,-1,0),
    3 : (0,0,1),
    4 : (1,0,0),
    5 : (0,1,0),
    6 : (0,0,-1)
}

class ScissorArm(robot):

    def __init__(self, robot_name, trajectory, gripper, multipart=False):
        robot.__init__(self, robot_name)
        self.mapping = MAPPING
        self.trajectory = trajectory
        self.idx = 0
        self.multipart = multipart
        self.gripper = gripper
        if multipart:
            pivots = [0]
            for traj in trajectory:
                pivots.append(pivots[-1] + len(traj))
            pivots = pivots[1:]
            traj = []
            for i in range(len(trajectory)):
                for j in range(len(trajectory[i])):
                    traj.append(trajectory[i][j])
            self.trajectory = traj
        self.preprocessing()

    def step(self):
        """
        Steps to the next position in the trajectory, cutting along the way.
        """
        if self.done:
            return False
        self.gripper.step(self.idx)
        self.open_gripper(80)
        time.sleep(2.5)
        if self.multipart:
            if self.idx + 1 in self.pivots:
                self.reenter()
        frame = get_frame(self.trajectory[self.idx+1], self.angles[self.idx+1])
        self.move_cartesian_frame_linear_interpolation(frame, 0.1)
        self.open_gripper(1)
        time.sleep(2.5)
        self.idx += 1
        if self.done:
            return False
        return True

    def reenter(self):
        """
        Reenters at the first index of the next trajectory. Needs to be implemented still.
        """
        return

    def preprocessing(self):
        all_angles = []
        factor = 2
        if self.multipart:
            for traj in self.trajectory:
                pts = np.array(traj)
                angles = []
                for i in range(pts.shape[0]-1):
                    pos = pts[i,:]
                    nextpos = pts[i+1,:]
                    angle = self.get_angle(np.ravel(pos), np.ravel(nextpos))
                    angles.append(angle)
                for i in range(len(angles)-2):
                    angles[i] = 0.5 * angles[i] + 0.35 * angles[i+1] + 0.15 * angles[i+2]
                angles = savgol_filter(angles, factor * (pts.shape[0]/12) + 1, 2)
                all_angles.extend(angles)
            self.angles = all_angles
        else:
            for traj in [self.trajectory]:
                pts = np.array(traj)
                angles = []
                for i in range(pts.shape[0]-1):
                    pos = pts[i,:]
                    nextpos = pts[i+1,:]
                    angle = self.get_angle(np.ravel(pos), np.ravel(nextpos))
                    angles.append(angle)
                for i in range(len(angles)-2):
                    angles[i] = 0.5 * angles[i] + 0.35 * angles[i+1] + 0.15 * angles[i+2]
                angles = savgol_filter(angles, factor * (pts.shape[0]/12) + 1, 2)
                all_angles.extend(angles)
            self.angles = all_angles




    def get_angle(self, pos, nextpos):
        """
        Returns angle to nextpos in degrees
        """
        delta = nextpos - pos
        theta = np.arctan(delta[1]/delta[0]) * 180 / np.pi
        if delta[0] < 0:
            return theta + 180
        return theta

    @property
    def done(self):
        return self.idx >= len(self.trajectory) - 2

class GripperArm(robot):

    # GRAB_ORIENTATION = (0.178626136475, 0.980532321834, -0.0781338284913, -0.0233275385155)
    GRAB_ORIENTATION = (0.300731240434, 0.861155549003, -0.324375232238, -0.250504591095)

    def __init__(self, robot_name, policy, scale=0.001):
        robot.__init__(self, robot_name)
    	self.initial_position = np.array(self.get_current_cartesian_position().position)
        self.mapping = MAPPING
        self.policy = policy
        self.scale = scale

    @property
    def displacement(self):
        """
        Computes the displacement of the tensioner from the original position.
        """
        return np.array(self.get_current_cartesian_position().position) - self.initial_position

    def cur_position_translation(self, translation):
        """
        Computes the final position vector after translating the current position by translation.
        """
        translation = np.array(translation)
        position = np.ravel(np.array(self.get_current_cartesian_position().position))
        return translation + position

    def execute_action(self, action, orientation=None):
        """
        Given a 3-tuple, execute the action associated with it on the robot.
        """
        if not orientation:
            self.move_cartesian_frame_linear_interpolation(tfx.pose(self.cur_position_translation(np.array(action) * self.scale), np.array(self.get_current_cartesian_position().orientation)), 0.1)
        else:
            self.move_cartesian_frame_linear_interpolation(tfx.pose(self.cur_position_translation(np.array(action) * self.scale), np.array(orientation)), 0.1)

    def query_policy(self, time):
        """
        Given a time index, the arm queries the trained policy for an action to take.
        """
        return self.mapping[self.policy.get_action(np.array([time]+list(self.displacement)))[0]]

    def step(self, time):
        """
        Queries the policy and executes the next action.
        """
        self.execute_action(self.query_policy(time))

    def grab_point(self, pos):
        """
        Grabs the gauze at pos.
        """
        self.move_cartesian_frame_linear_interpolation(tfx.pose(np.array(pos), np.array(self.GRAB_ORIENTATION)), 0.1)
        self.grab_current_point()



    def grab_current_point(self):
        """
        Grabs directly below the current location.
        """
        self.open_gripper(80)
        time.sleep(2.5)
        self.execute_action((0, 0, -10), self.GRAB_ORIENTATION)
        self.open_gripper(-30)
        time.sleep(2.5)
        self.execute_action((0, 0, 10), self.GRAB_ORIENTATION)

def get_frame(pos, angle, offset=0.003):
    """
    Given a position and an orientation, compute a tfx frame characterizing the pose.
    """
    angle = angle
    pos = np.array(pos)
    pos[2] -= offset
    rotation = [94.299363207+angle, -4.72728031036, 86.1958002688]
    rot = tfx.tb_angles(rotation[0], rotation[1], rotation[2])
    frame = tfx.pose(pos, rot)
    return frame
