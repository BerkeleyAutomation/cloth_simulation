from robot import *
import sys, os, pickle, copy, time
import numpy as np
import tfx
import os.path as osp
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
sys.path.append("home/davinci0/cloth_simulation")
import notch

MAPPING = {
    0 : (0,0,0),
    1 : (-0.75,0,0),
    2 : (0,0.75,0),
    3 : (0,0,0),
    4 : (0.75,0,0),
    5 : (0,-0.75,0),
    6 : (0,0,0) #-
}

class ScissorArm(robot):

    def __init__(self, robot_name, trajectory, gripper, multipart=True):
        robot.__init__(self, robot_name)
        self.mapping = MAPPING
        self.trajectory = trajectory
        self.idx = 0
        self.multipart = multipart
        self.gripper = gripper
        self.preprocessing()
        if multipart:
            pivots = [0]
            for traj in trajectory:
                pivots.append(pivots[-1] + len(traj))
            pivots = pivots[:]
            traj = []
            for i in range(len(trajectory)):
                for j in range(len(trajectory[i])):
                    traj.append(trajectory[i][j])
            self.trajectory = traj
            self.pivots = pivots
            # print pivots; sys.exit()

        self.lock = 0
        

    def step(self, blobs=None):
        """
        Steps to the next position in the trajectory, cutting along the way.
        """
        if self.done:
            return False
        if self.idx%2 == 0:
            self.gripper.step(self.idx/2, blobs)
        self.open_gripper(80)
        time.sleep(2.5)
        if self.multipart:
            if self.idx + 1 in self.pivots or self.idx == 0:
                reenterpos = self.trajectory[self.idx+1]
                if self.idx != 0:
                    # reenterpos = self.trajectory[0]
                    pass
                self.reenter(reenterpos)
        pos = self.trajectory[self.idx+1]
        if self.idx+2 in self.pivots or self.idx+1 >= len(self.trajectory) or self.idx+1 >= len(self.angles):
            angle = self.angles[self.idx]
            frame = get_frame(pos, self.angles[self.idx])
        else:
            frame = get_frame(pos, self.angles[self.idx+1])
            angle = self.angles[self.idx+1]
            print self.angles[self.idx+1]
        print self.idx, frame
        self.move_cartesian_frame_linear_interpolation(frame, 0.04)
        self.open_gripper(1)
        time.sleep(2.5)
        if self.lock > 0:
            self.lock -= 1
            frame = get_frame(np.ravel(self.get_current_cartesian_position().position) + np.array([0,0,0.003]), angle)
            self.move_cartesian_frame_linear_interpolation(frame, 0.1)
            time.sleep(2)
            self.open_gripper(80)
            time.sleep(2)
            frame = get_frame(np.ravel(self.get_current_cartesian_position().position) + np.array([0,0,-0.003]), angle)
            self.move_cartesian_frame_linear_interpolation(frame, 0.1)
            time.sleep(2)
            self.open_gripper(1)
            time.sleep(2.5)
        self.idx += 1
        if self.done:
            return False
        return True

    def reenter(self, pt):
        """
        Reenters at the first index of the next trajectory. Needs to be implemented still.
        """
        if self.idx < 10:
            angle = 0
        else:
            angle = -80
            self.lock = 3
        self.open_gripper(-15)
        time.sleep(2)
        frame = get_frame(np.ravel(self.get_current_cartesian_position().position) + np.array([0,0.018,0.01]), 0)
        self.move_cartesian_frame_linear_interpolation(frame, 0.1)
        time.sleep(2)
        self.home()
        time.sleep(1)
        self.gripper.reset()
        pt = np.array(pt)
        pt[0] -= 0.00
        pt[2] += 0.0035
        print pt
        notch.cut_notch_angle(pt, self, angle)
        time.sleep(2)
        # self.gripper.execute_action((0, 0, 2))
        frame = tfx.pose(np.ravel(self.get_current_cartesian_position().position) + np.array([0,0,0.005]), np.array(self.get_current_cartesian_position().orientation))
        self.move_cartesian_frame_linear_interpolation(frame, 0.1)
        time.sleep(2)
        frame = get_frame(np.ravel(self.get_current_cartesian_position().position), -80)
        self.move_cartesian_frame_linear_interpolation(frame, 0.04)
        time.sleep(2)
        self.open_gripper(-15)
        time.sleep(2)
        self.open_gripper(75)
        time.sleep(2)
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
                if len(angles) > 15:
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

    def home(self):
        print "HOMING"
        frame = get_frame([0.0402655955015, 0.0348254948724, -0.0667273747345], 0)
        self.move_cartesian_frame_linear_interpolation(frame, 0.1)
        time.sleep(2)

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
        print action
        if not orientation:
            self.move_cartesian_frame_linear_interpolation(tfx.pose(self.cur_position_translation(np.array(action) * self.scale), np.array(self.get_current_cartesian_position().orientation)), 0.1)
        else:
            self.move_cartesian_frame_linear_interpolation(tfx.pose(self.cur_position_translation(np.array(action) * self.scale), np.array(orientation)), 0.1)

    def query_policy(self, time, blobs=None):
        """
        Given a time index, the arm queries the trained policy for an action to take.
        """
        print np.array([time]+list(self.displacement) + list(blobs)).shape
        action = self.mapping[self.policy.get_action([time]+list(self.displacement) + list(blobs))[0]]
        return action

    def step(self, time, blobs):
        """
        Queries the policy and executes the next action.
        """
        self.execute_action(self.query_policy(time, blobs))

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
        self.execute_action((0, 0, -15), self.GRAB_ORIENTATION)
        self.open_gripper(-30)
        time.sleep(2.5)
        self.execute_action((0, 0, 15), self.GRAB_ORIENTATION)
        time.sleep(2.5)
        self.initial_position = np.array(self.get_current_cartesian_position().position)
        print self.initial_position

    def reset(self):
        disp = -np.ravel(self.displacement)
        disp = self.cur_position_translation(disp)
        print "DISP", np.ravel(disp)
        self.move_cartesian_frame_linear_interpolation(tfx.pose(np.array(disp), np.array(self.get_current_cartesian_position().orientation)), 0.1)

def get_frame(pos, angle, offset=0.0035):
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
