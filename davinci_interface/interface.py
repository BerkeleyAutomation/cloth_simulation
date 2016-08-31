from robot import *
import sys, os, pickle, copy
import numpy as np
import tfx

MAPPING = {
    0 : (0,0,0),
    1 : (1,0,0),
    2 : (0,1,0),
    3 : (0,0,1),
    4 : (-1,0,0),
    5 : (0,-1,0),
    6 : (0,0,-1)
}

class ScissorArm(robot):

    def __init__(self, robot_name, trajectory, gripper):
        robot.__init__(self, robot_name)
        self.mapping = MAPPING
        self.trajectory = trajectory
        self.idx = 0
        self.gripper = gripper

    def step(self):
        """
        Steps to the next position in the trajectory, cutting along the way.
        """
        if self.done:
            return False
        self.gripper.step(self.idx)
        self.open_gripper(80)
        self.move_cartesian_frame_linear_interpolation(tfx.pose(self.trajectory[self.idx+1], np.array(self.get_current_cartesian_position().orientation)))
        self.open_gripper(1)
        self.idx += 1
        if self.done:
            return False
        return True

    @property
    def done(self):
        return idx == len(trajectory) - 2

class GripperArm(robot):

    def __init__(self, robot_name, policy, scale=0.01):
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
        position = np.array(self.get_current_cartesian_position().position)
        return translation + position

    def execute_action(self, action):
        """
        Given a 3-tuple, execute the action associated with it on the robot.
        """
        self.move_cartesian_frame_linear_interpolation(tfx.pose(cur_position_translation(action * self.scale), np.array(self.get_current_cartesian_position().orientation)), 0.1)

    def query_policy(self, time):
        """
        Given a time index, the arm queries the trained policy for an action to take.
        """
        return self.mapping[query_policy(policy, [i]+list(self.displacement))[0]]

    def step(self, time):
        execute_action(query_policy(time))
