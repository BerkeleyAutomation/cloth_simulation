import sys, os, pickle, time
import matplotlib.pyplot as plt
import numpy as np
sys.path.append(os.path.dirname(os.getcwd()))
from calibration.robot import *
from numpy import array



if __name__ == '__main__':
  
    ORIENTATION = np.array([0.902174198372, 0.208355564829, -0.240495636706, -0.291258515964])
    HOME = np.array([0.0195990646306, 0.0274673676628, -0.0720500382531])

    psm1 = robot("PSM1")
    pts3d = [[0.0652561137574198, 0.07532958469798597, -0.11921077140820625], [0.08471072999415775, 0.0769386782489475, -0.122362128756043], [0.045312281495582196, 0.04965118268862282, -0.12002638452539917], [0.0644238949572248, 0.05067063408971723, -0.12232268647988047], [0.10382787149808262, 0.05431112648862298, -0.12278520126581208], [0.08400885644510828, 0.05144546152994814, -0.12298126851562856]]
    pts3d = [array([ 0.09204918,  0.06863853, -0.12463619]), array([ 0.05975166,  0.06704997, -0.12074739]), array([ 0.09204918,  0.06863853, -0.12463619]), array([ 0.06178487,  0.05986307, -0.11972547]), array([ 0.05310647,  0.05487145, -0.11895887]), array([ 0.09712256,  0.0564491 , -0.12263689]), array([ 0.08458751,  0.04944951, -0.12317438]), array([ 0.07077065,  0.04636858, -0.12250585]), array([ 0.06019233,  0.04472778, -0.12216285])]

    for pt in pts3d:
        print pt
        pt = list(pt)
        psm1.move_cartesian_frame_linear_interpolation(tfx.pose(pt, ORIENTATION), 0.1)
        time.sleep(1)
        psm1.move_cartesian_frame_linear_interpolation(tfx.pose(HOME, ORIENTATION), 0.1)
        time.sleep(1)
