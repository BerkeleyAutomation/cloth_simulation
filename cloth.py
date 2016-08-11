import matplotlib.pyplot as plt
import numpy as np
from math import sqrt
import sys, pickle

"""
This file contains classes and functions used to simulate the basic physics of a simplified 3D cloth model.
"""

"""
A class that simulates a point mass. A cloth is made up of a collection of these interacting with each other.
"""
class Point(object):

    def __init__(self, mouse, x=0, y=0, z=0):
        """
        Initializes an instance of a particle.
        """
        self.mouse = mouse
        self.x, self.y, self.z = x, y, z
        self.px, self.py, self.pz = x, y, z
        self.vx, self.vy, self.vz = 0, 0, 0
        self.constraints = []
        self.pinned = False

    def add_constraint(self, pt):
        """
        Adds a constraint between this point and another point.
        """
        self.constraints.append(Constraint(self, pt))

    def add_force(self, x, y, z=0):
        """
        Applies a force to itself.
        """
        if not self.pinned:
            self.vx, self.vy, self.vz = self.vx + x, self.vy + y, self.vz + z

    def resolve_constraints(self):
        """
        Resolves all constraints pertaining to this point, and simulates bouncing off the walls if the point tries to go out of bounds.
        """
        for constraint in self.constraints:
            constraint.resolve()
        boundsx = 600
        boundsy = 600
        boundsz = 800
        if self.x >= boundsx:
            self.x = 2 * boundsx - self.x
        elif self.x < 1:
            self.x = 2 - self.x
        if self.y >= boundsy:
            self.y = 2 * boundsy - self.y
        elif self.y < 1:
            self.y = 2 - self.y
        if self.z >= boundsz:
            self.z = 2 * boundsz - self.z
        elif self.z <= -boundsz:
            self.z = -2 * boundsz - self.z

    def update(self, delta):
        """
        Updates the point, takes in mouse input. Applies a gravitational force to it, this parameter can be tuned for varying results.
        """
        if self.mouse.down:
            dx, dy, dz = self.x - self.mouse.x, self.y - self.mouse.y, self.z - self.mouse.z
            dist = sqrt(dx ** 2 + dy ** 2)

            if self.mouse.button == 1:
                if dist < self.mouse.influence:
                    self.px = self.x - (self.mouse.x - self.mouse.px) * 1.8
                    self.py = self.y - (self.mouse.y - self.mouse.py) * 1.8
            elif dist < self.mouse.cut and abs(dz) < self.mouse.height_limit:
                self.constraints = []

        # gravity parameter, increase magnitude to increase gravity
        gravity = -1000
        self.add_force(0, 0, gravity)
        delta *= delta

        nx = self.x + ((self.x - self.px)) * 0.99 + ((self.vx / 2.0) * delta)
        ny = self.y + ((self.y - self.py)) * 0.99 + ((self.vy / 2.0) * delta)
        nz = self.z + ((self.vz / 2.0) * delta)

        self.px, self.py, self.pz = self.x, self.y, self.z
        self.x, self.y, self.z = nx, ny, nz
        self.vx, self.vy, self.vz = 0, 0, 0

"""
A class to represent interactions between Points.
"""
class Constraint(object):

    def __init__(self, p1=None, p2=None, tear_dist=100):
        """
        Constraint between two points that attempts to maintain a fixed distance between points and tears if a threshold is passed.
        """
        self.p1, self.p2 = p1, p2
        self.length = sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
        self.tear_dist = tear_dist

    def resolve(self):
        """
        Updates the points in the constraint based on how much the constraint has been violated. Elasticity is a paramter that can be tuned that affects the response of a constraint.
        """
        dx, dy, dz = self.p1.x - self.p2.x, self.p1.y - self.p2.y, self.p1.z - self.p2.z
        dist = sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        diff = (self.length - dist) / float(dist)

        if dist > self.tear_dist:
            self.p1.constraints.remove(self)

        # Elasticity, usually pick something between 0.01 and 1.5
        elasticity = 1

        px, py, pz = [delta * diff * 0.5 * elasticity for delta in (dx, dy, dz)]

        if not self.p1.pinned:
            self.p1.x, self.p1.y, self.p1.z = self.p1.x + px, self.p1.y + py, self.p1.z + pz

        if not self.p2.pinned:
            self.p2.x, self.p2.y, self.p2.z = self.p2.x - px, self.p2.y - py, self.p2.z - pz

"""
A cloth class, which consists of a collection of points and their corresponding constraints.
"""
class Cloth(object):

    def __init__(self, mouse, width=50, height=50, dx=10, dy=10):
        """
        Creates a cloth with width x height points spaced dx and dy apart. The top and bottom row of points are pinned in place.
        """
        self.mouse = mouse
        self.pts = []
        for i in range(height):
            for j in range(width):
                pt = Point(mouse, 50 + dx * j, 50 + dy * i)
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pass
                    pt.add_constraint(self.pts[-1])
                if i == height - 1 or i == 0:
                    pt.pinned = True
                self.pts.append(pt)

    def update(self):
        """
        Updates all the points in the cloth based on existing constraints. If a point exists with no constraints, remove it from the cloth.
        """
        # Setting this to 5 is pretty decent, probably don't need to increase it
        physics_accuracy = 5
        for i in range(physics_accuracy):
            for pt in self.pts:
                pt.resolve_constraints()
        for pt in self.pts:
            pt.update(0.016)
        for pt in self.pts:
            if pt.constraints == []:
                self.pts.remove(pt)
"""
A subclass of cloth, on which a circle pattern is drawn. It also can be grabbed and tensioned.
"""
class CircleCloth(Cloth):

    def __init__(self, mouse, width=50, height=50, dx=10, dy=10, centerx=300, centery=300, radius=150):
        """
        A cloth on which a circle can be drawn. It can also be grabbed and tensioned at specific coordinates.
        """
        self.pts = []
        self.circlepts = []
        self.normalpts = []
        self.grabbed_pts = []
        self.mouse = mouse
        for i in range(height):
            for j in range(width):
                pt = Point(mouse, 50 + dx * j, 50 + dy * i)
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pass
                    pt.add_constraint(self.pts[-1])
                if i == height - 1 or i == 0:
                    pt.pinned = True
                if abs((pt.x - centerx) **2 + (pt.y - centery) ** 2 - radius **2) < 2000:
                    self.circlepts.append(pt)
                else:
                    self.normalpts.append(pt)
                self.pts.append(pt)

    def update(self):
        """
        Update function updates the state of the cloth after a time step.
        """
        physics_accuracy = 5
        for i in range(physics_accuracy):
            for pt in self.pts:
                pt.resolve_constraints()
        for pt in self.pts:
            pt.update(0.016)
        for pt in self.pts:
            if pt.constraints == []:
                self.pts.remove(pt)
                if pt in self.circlepts:
                    self.circlepts.remove(pt)
                else:
                    self.normalpts.remove(pt)

    def pin_position(self, x, y):
        """
        Grab a position on the cloth and pin it in place.
        """
        for pt in self.pts:
            if abs((pt.x - x) ** 2 + (pt.y - y) ** 2) < 1000:
                pt.pinned = True
                self.grabbed_pts.append(pt)

    def unpin_position(self, x, y):
        """
        Let go of a grabbed position.
        """
        if abs((pt.x - x) ** 2 + (pt.y - y) ** 2) < 1000:
            pt.pinned = False
            self.grabbed_pts.remove(pt)

    def tension(self, x, y, z=0):
        """
        Tug on the grabbed area in a direction.
        """
        for pt in self.grabbed_pts:
            pt.px, pt.py, pt.pz = pt.x, pt.y, pt.z
            pt.x, pt.y, pt.z = x + pt.x, y + pt.y, z + pt.z

"""
An implementation of a mouse class, that can be updated/modified to cut the cloth or disturb it. This can be used to interface with a physical or virtual mouse.
"""
class Mouse(object):

    def __init__(self, x=0, y=0, z=0, height_limit=False):
        self.down = False
        self.button = 0
        self.x, self.y, self.z = x, y, z
        self.px, self.py, self.pz = x, y, z
        self.cut = 10
        self.influence = 5
        if height_limit:
            self.height_limit = height_limit
        else:
            self.height_limit = float('inf')

    def move(self, x, y):
        """
        Move mouse to a position on the canvas.
        """
        self.px, self.py = self.x, self.y
        self.x, self.y = x, y

    def clicked(self, event):
        """
        Handles click events of the mouse.
        """
        self.down = True

    def released(self, event):
        """
        Handles mouse release events.
        """
        self.down = False

    def moved(self, event):
        """
        Handles mouse move events.
        """
        self.move(event.xdata, event.ydata)

### UTILITY FUNCTIONs ###

def write_to_file(cloth, filename):
    """
    Write cloth's state to file.
    """
    f = open(filename, "w+")
    pickle.dump(cloth, f)
    f.close()

def load_from_file(filename):
    """
    Load a past state from file.
    """
    f=open(filename,'rb')
    try:
        return pickle.load(f)
    except EOFError:
        print 'Nothing written to file.'

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
        cpts = np.array([[p.x, p.y] for p in c.circlepts])
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
