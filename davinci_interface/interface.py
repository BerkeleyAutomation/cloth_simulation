from robot import *
import sys, os, pickle, copy, time
import numpy as np
import tfx
import os.path as osp

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
        self.move_cartesian_frame_linear_interpolation(tfx.pose(self.trajectory[self.idx+1], np.array(self.get_current_cartesian_position().orientation)), 0.1)
        self.open_gripper(1)
        time.sleep(2.5)
        self.idx += 1
        if self.done:
            return False
        return True

    def reenter(self):
        """
        Reenters at the first index of the next trajecory. Needs to be implemented still.
        """
        return

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
        self.open_gripper(80)
        time.sleep(2.5)
        self.execute_action((0, 0, -10), self.GRAB_ORIENTATION)
        self.open_gripper(-30)
        time.sleep(2.5)
        self.execute_action((0, 0, 10), self.GRAB_ORIENTATION)
