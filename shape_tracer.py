import Tkinter
from Tkinter import *
import rospy, pickle, time
from geometry_msgs.msg import Pose
import multiprocessing
import numpy as np
import sys, os
import os.path as osp

"""
Launching this script creates a GUI that subscribes to PSM1's position_cartesian_current topic and can write this information to file.
"""

def startCallback():
    global prs
    process = multiprocessing.Process(target = start_listening)
    prs.append(process)
    process.start()
    return

def start_listening():
    global sub
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('/dvrk/PSM1/position_cartesian_current', Pose, callback_PSM1_actual)
    rospy.spin()

def exitCallback():
    global prs
    for process in prs:
        process.terminate()
    plot_points()
    sys.exit()

def callback_PSM1_actual(data):
    position = data.position
    psm1_pose = [position.x, position.y, position.z]
    print psm1_pose
    f = open(osp.join(directory, gauze_pts+'.p'), "a")
    pickle.dump(psm1_pose, f)
    f.close()
    sub.unregister()

def load_robot_points(fname="gauze_pts.p"):
    lst = []
    f3 = open(osp.join(directory, fname),"rb")
    while True:
        try:
            pos2 = pickle.load(f3)
            lst.append(pos2)
        except EOFError:
            f3.close()
            return np.matrix(lst)

def plot_points():
    """
    Plots points in robot_frame. Axes may need to be edited.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    pts=load_robot_points()
    if pts.shape[1] == 0:
        print "no points to show"
        return
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(pts[:,0]), np.array(pts[:,1]), np.array(pts[:,2]),c='r')
    ax.set_xlim3d(0, 0.2)
    ax.set_ylim3d(-0.1, 0.1)
    ax.set_zlim3d(-0.15,0.05)
    plt.show()

def switchCallback():
    global gauze_pts
    plot_points()
    gauze_pts='gauze_pts2'

if __name__ == '__main__':

    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        directory = "calibration_data"
    sub = None
    prs = []
    gauze_pts='gauze_pts'

    if not os.path.exists(directory):
        os.makedirs(directory)

    open(osp.join(directory, 'gauze_pts.p'), 'w+').close()
    open(osp.join(directory, 'gauze_pts2.p'), "w+").close()
    top = Tkinter.Tk()
    top.title('Calibration')
    top.geometry('400x200')

    B = Tkinter.Button(top, text="Record Position PSM1", command = startCallback)
    D=Tkinter.Button(top, text="part 2", command = switchCallback)
    F = Tkinter.Button(top, text="Exit", command = exitCallback)

    B.pack()
    D.pack()
    F.pack()
    top.mainloop()
