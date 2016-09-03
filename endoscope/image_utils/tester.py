import sys, os, pickle, time
import matplotlib.pyplot as plt
import numpy as np
sys.path.append(os.path.dirname(os.getcwd()))
from calibration.robot import *



if __name__ == '__main__':
  
    ORIENTATION = np.array([0.902174198372, 0.208355564829, -0.240495636706, -0.291258515964])
    HOME = np.array([0.0195990646306, 0.0274673676628, -0.0720500382531])

    psm1 = robot("PSM1")
    pts3d = [[0.052698558899129416, 0.07994646981647706, -0.11737959846372095], [0.05820706050470008, 0.04114254219306345, -0.12301286110192933], [0.10383344738653913, 0.08332514181450618, -0.11790044675120354], [0.10850323671938711, 0.04715575210408738, -0.12379845812505619]]

    for pt in pts3d:
        print pt
        psm1.move_cartesian_frame_linear_interpolation(tfx.pose(pt, ORIENTATION), 0.1)
        time.sleep(1)
        psm1.move_cartesian_frame_linear_interpolation(tfx.pose(HOME, ORIENTATION), 0.1)
        time.sleep(1)
