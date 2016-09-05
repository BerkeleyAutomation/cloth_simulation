import numpy as np
import matplotlib.pyplot as plt
import pickle, copy, sys
import json
from cloth import *
from circlecloth import *
from shapecloth import *
from tensioner import *
from mouse import *
from registration import *
from scorer import *
import IPython
from notch_finder import *


"""
A Simulation object that can be used to represent an ongoing experiment. It can be rendered by setting render=True on construction. See the main method for an example.
"""
class Simulation(object):

    def __init__(self, cloth, init=200, render=False, update_iterations=1, trajectory=None, multi_part=False):
        """
        Constructor takes in a cloth object and optionally, a nonnegative integer representing the amount of time to spend allowing
        the cloth to settle initially. Setting render=True will render the simulation. However, rendering will slow down iterations 
        by approximately 5x.
        """
        self.cloth = cloth
        self.mouse = self.cloth.mouse
        self.tensioners = self.cloth.tensioners
        self.render = render
        self.init = init
        self.bounds = cloth.bounds
        self.stored = False
        self.update_iterations = update_iterations
        self.trajectory = trajectory
        if not trajectory:
            self.trajectory = [(np.cos(deg) * 150 + 300, np.sin(deg) * 150 + 300) for deg in [3.6 * np.pi * i / 180.0 for i in range(100)]]
        self.multi_part = multi_part
        traj = []
        if multi_part:
            for i in range(len(trajectory)):
                for j in range(len(trajectory[i])):
                    traj.append(trajectory[i][j])
            self.trajectory = traj



    def update(self, iterations=-1):
        """
        Updates the state of the cloth. Iterations signifies the amount of time to spend to allow the cloth to equilibrate.
        """
        if iterations < 0:
            iterations = self.update_iterations
        ret = sum([self.cloth.update() for _ in range(iterations)])
        if self.render:
            self.render_sim()
        return ret

    def render_sim(self):
        plt.clf()
        pts = np.array([[p.x, p.y] for p in self.cloth.normalpts])
        cpts = np.array([[p.x, p.y] for p in self.cloth.shapepts])
        bpts = []
        for blob in self.cloth.blobs:
            for pt in blob:
                bpts.append([pt.x, pt.y])
        bpts = np.array(bpts)
        if len(pts) > 0:
            plt.scatter(pts[:,0], pts[:,1], c='w')
        if len(cpts) > 0:
            plt.scatter(cpts[:,0], cpts[:,1], c='b')
        if len(bpts) > 0:
            plt.scatter(bpts[:,0], bpts[:,1], c='r')
        ax = plt.gca()
        plt.axis([0, 600, 0, 600])
        ax.set_axis_bgcolor('white')
        plt.pause(0.01)

    def pin_position(self, x, y, max_displacement=False):
        """
        Pins a position on the cloth.
        """
        return self.cloth.pin_position(x, y, max_displacement)

    def unpin_position(self, x, y):
        """
        Unpins a previously pinned position on the cloth.
        """
        self.cloth.unpin_position(x, y)

    def move_mouse(self, x, y):
        """
        Moves the mouse object.
        """
        self.mouse.move(x, y)

    def reset(self):
        """
        Resets the simulation object.
        """
        print "Resetting simulation."
        if not self.stored:
            self.cloth.reset()
            self.mouse = self.cloth.mouse
            self.tensioners = self.cloth.tensioners
            print "Initializing cloth"
            for i in range(self.init):
                self.cloth.update()
                if i % 10 == 0:
                    print str(i) + '/' + str(self.init)
            self.stored = copy.deepcopy(self.cloth)
            self.update(0)
        else:
            self.cloth = copy.deepcopy(self.stored)
            self.mouse = self.cloth.mouse
            self.tensioners = self.cloth.tensioners
            self.bounds = self.cloth.bounds
            self.update(0)

    def write_to_file(self, fname):
        """
        Writes a simulation object to file.
        """
        f = open(fname, "w+")
        pickle.dump(self, f)
        f.close()

    def read_from_file(fname):
        """
        Load a simuation object from file.
        """
        f = open(fname, "rb")
        try:
            return pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'


    # def __deepcopy__(self):
    #     """
    #     Returns a deep copy of self.
    #     """
    #     return copy.deepcopy(self)

def load_simulation_from_config(fname="config_files/default.json", shape_fn=None, trajectory=None, multipart=False):
    """
    Creates a Simulation object from a configuration file FNAME, and can optionally take in a SHAPE_FN or create one from discrete points saved to file. MULTIPART indicates whether or not the input trajectory consists of multiple subtrajectories.
    """
    with open(fname) as data_file:    
        data = json.load(data_file)
    mouse = data["mouse"]
    bounds = data["bounds"]
    bounds = (bounds["x"], bounds["y"], bounds["z"])
    mouse = Mouse(mouse["x"], mouse["y"], mouse["z"], mouse["height_limit"], mouse["down"], mouse["button"], bounds, mouse["influence"], mouse["cut"])
    cloth = data["shapecloth"]
    corners, blobs = None, None
    if "blobs" in data["options"].keys():
        corners = load_robot_points(data["options"]["blobs"][0])
        blobs = load_points(data["options"]["blobs"][1])
    if not shape_fn:
        corners = load_robot_points(cloth["shape_fn"][0])
        pts = load_robot_points(cloth["shape_fn"][1])
        shape_fn = get_shape_fn(corners, pts, True)
        if not trajectory:
            trajectory = load_trajectory_from_config(fname)
    cloth = ShapeCloth(shape_fn, mouse, cloth["width"], cloth["height"], cloth["dx"], cloth["dy"], 
        cloth["gravity"], cloth["elasticity"], cloth["pin_cond"], bounds, blobs, corners)
    simulation = data["simulation"]
    if "multipart" in simulation.keys() and not multipart:
        multipart = simulation["multipart"]

        if multipart:
            # Find the notch points and segments to complete the trajectory
            npf = NotchPointFinder(cloth, trajectory)
            npf.find_pts("r") # cutting from right "r"
            npf.find_segments("r")
            from scorer import *
            scorer = Scorer(0)
            trajectory = npf.find_best_trajectory(scorer) # trajectory is now a list of lists
            IPython.embed()
    return Simulation(cloth, simulation["init"], simulation["render"], simulation["update_iterations"], trajectory, multipart)

def load_trajectory_from_config(fname="config_files/default.json"):
    """
    Returns a trajectory created from the pt registration files specified in FNAME.
    """
    with open(fname) as data_file:    
        data = json.load(data_file)
    cloth = data["shapecloth"]
    corners = load_robot_points(cloth["shape_fn"][0])
    pts = load_robot_points(cloth["shape_fn"][1])
    return get_trajectory(corners, pts, True)

def load_pin_from_config(fname="config_files/default.json"):
    """
    Returns a pin position from a config file FNAME.
    """
    with open(fname) as data_file:    
        data = json.load(data_file)
    options = data["options"]
    pin = options["pin_position"]
    option = options["max_displacement"]
    x = pin["x"]
    y = pin["y"]
    if not any((x, y)):
        return None
    return (x, y), option

def read_trajectory_from_file(fname):
    """
    Load a trajectory from file.
    """
    with open(fname, "rb") as f:
        try:
            return pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'

def write_trajectory_to_file(trajectory, fname):
    """
    Writes a trajectory to file.
    """
    with open(fname, "w+") as f:
        pickle.dump(trajectory, f)


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
    else:
        shape_fn=None

    simulation = load_simulation_from_config(shape_fn=shape_fn)
    scorer = Scorer(0)
    simulation.reset()

    print "Initial Score", scorer.score(simulation.cloth)
    print len(simulation.trajectory)

    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])

    print "Score", scorer.score(simulation.cloth)
    
    simulation.reset()
    pin_position, option = load_pin_from_config()
    print pin_position
    if pin_position:
        simulation.pin_position(pin_position[0], pin_position[1], option)

    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])    

    print "Score", scorer.score(simulation.cloth)

