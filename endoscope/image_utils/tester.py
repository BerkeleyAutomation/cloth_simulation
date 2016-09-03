import sys, os, pickle, time
import matplotlib.pyplot as plt
import numpy as np
sys.path.append(os.path.dirname(os.getcwd()))
from calibration.robot import *



if __name__ == '__main__':
  
    ORIENTATION = np.array([0.902174198372, 0.208355564829, -0.240495636706, -0.291258515964])
    HOME = np.array([0.0195990646306, 0.0274673676628, -0.0720500382531])

    psm1 = robot("PSM1")
    pts3d = [[0.052581410638160984, 0.08031761588982435, -0.11819119174406409], [0.05726729566445271, 0.040802760157315224, -0.12204121675152378], [0.10801663273917529, 0.04734514599972541, -0.12214490730812565]]

    for pt in pts3d:
        print pt
        psm1.move_cartesian_frame_linear_interpolation(tfx.pose(pt, ORIENTATION), 0.1)
        time.sleep(1)
        psm1.move_cartesian_frame_linear_interpolation(tfx.pose(HOME, ORIENTATION), 0.1)
        time.sleep(1)
