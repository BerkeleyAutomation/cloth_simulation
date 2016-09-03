import cv2
import Tkinter
from Tkinter import *
import rospy, pickle, time
from geometry_msgs.msg import Pose
import multiprocessing
import numpy as np
import sys
from robot import *
import tfx


"""
This script contains functions and a GUI used for collecting points in robot frame (for PSM1). It dumps points to calibration_data/psm1_calibration.
"""



def exitCallback():
    with open("camera_data/psm1_calibration.p", "w+") as f:
        pickle.dump(psm1_pts, f)
    with open("camera_data/psm2_calibration.p", "w+") as f:
        pickle.dump(psm2_pts, f)
    sys.exit()

def psm1_callback():
    print list(psm1.get_current_cartesian_position().position)
    psm1_pts.append(list(psm1.get_current_cartesian_position().position))

def psm2_callback():
    print list(psm1.get_current_cartesian_position().position)
    psm2_pts.append(list(psm2.get_current_cartesian_position().position))


if __name__ == '__main__':
    open('camera_data/psm1_calibration.p', 'w+').close()
    open('camera_data/psm2_calibration.p', 'w+').close()
    psm1 = robot("PSM1")
    psm2 = robot("PSM2")
    top = Tkinter.Tk()
    top.title('Calibration')
    top.geometry('400x200')
    B = Tkinter.Button(top, text="Record PSM1 Position", command = psm1_callback)
    C = Tkinter.Button(top, text="Record PSM2 Position", command = psm2_callback)
    D = Tkinter.Button(top, text="Exit", command = exitCallback)
    [a.pack() for a in B, C, D]
    psm1_pts, psm2_pts = [], []
    top.mainloop()
