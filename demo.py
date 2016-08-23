import pyximport
pyximport.install()

import matplotlib.pyplot as plt
import numpy as np
import sys
from cloth import *
from circlecloth import *
from mouse import *
from point import *
from constraint import *
from util import *

"""
This script contains a demo that can be run out of the box. If provided with the manual argument, the user can control the position of the cutting tool.
"""
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "manual":
        print "Manual cutting"
        auto = False
    else:
        print "Automated cutting"
        auto = True

    mouse = Mouse()
    mouse.down = True
    mouse.button = 0

    circlex = 300
    circley = 300
    radius = 150

    c = CircleCloth(mouse)

    # Let the cloth reach equilibrium"
    for i in range(200):
        c.update()
        if i % 10 == 0:
            print "Iteration", i

    # Simulate grabbing the gauze
    c.pin_position(circlex, circley)

    plt.ion()

    if not auto:
        fig = plt.figure()
        plot = fig.add_subplot(111)
        plot.set_title('manual')
        cid=fig.canvas.mpl_connect('button_press_event', mouse.clicked)
        rid=fig.canvas.mpl_connect('button_release_event', mouse.released)
        mid=fig.canvas.mpl_connect('motion_notify_event', mouse.moved)
    
    for i in range(400):
        if i % 10 == 0:
            print "Iteration", i
        plt.clf()
        pts = np.array([[p.x, p.y] for p in c.normalpts])
        cpts = np.array([[p.x, p.y] for p in c.shapepts])
        if len(pts) > 0:
            plt.scatter(pts[:,0], pts[:,1], c='w')
        if len(cpts) > 0:
            plt.scatter(cpts[:,0], cpts[:,1], c='b')
        ax = plt.gca()
        plt.axis([0, 600, 0, 600])
        ax.set_axis_bgcolor('white')
        plt.pause(0.01)
        c.update()
        
        # Extra updates to allow cloth to respond to environment.
        for j in range(5):
            c.update()

        # simulate moving the mouse in a circle while cutting, overcut since no perception

        if auto:
            if i < 150:
                theta = 360.0/100.0 * i * np.pi / 180.0
                x = radius * np.cos(theta)
                y = radius * np.sin(theta)
                mouse.move(x + circlex, y + circley)

    if not auto:
        fig.canvas.mpl_disconnect(cid)
        fig.canvas.mpl_disconnect(mid)
        fig.canvas.mpl_disconnect(rid)