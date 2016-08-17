from point import *
from mouse import *
from tensioner import *

"""
A cloth class, which consists of a collection of points and their corresponding constraints.
"""
class Cloth(object):

    def __init__(self, mouse=None, width=50, height=50, dx=10, dy=10, gravity=-1000.0, elasticity=1.0, pin_cond="default", bounds=(600, 600, 800)):
        """
        Creates a cloth with width x height points spaced dx and dy apart. The top and bottom row of points are pinned in place.
        """
        if not mouse:
            mouse = Mouse(bounds=bounds)
        self.mouse = mouse
        self.pts = []
        self.tensioners = []
        self.shapepts = []
        self.bounds = bounds
        if pin_cond == "default":
            pin_cond = lambda x, y, height, width: y == height - 1 or y == 0
        for i in range(height):
            for j in range(width):
                pt = Point(mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity, bounds=bounds)
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pt.add_constraint(self.pts[-1])
                if pin_cond(j, i, height, width):
                    pt.pinned = True
                self.pts.append(pt)
        self.initial_params = [(width, height), (dx, dy), gravity, elasticity, pin_cond]

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

    def add_tensioner(self, tensioner):
        self.tensioners.append(tensioner)

    def pin_position(self, x, y):
        """
        Grab a position on the cloth and pin it in place. Automatically adds it to the cloth's list of tensioners.
        """
        return Tensioner(x, y, self)

    def unpin_position(self, x, y):
        """
        Let go of a position held by a tensioner.
        """
        for tensioner in tensioners:
            if tensioner.x == x and tensioner.y == y:
                self.tensioners.remove(tensioner)
                tensioner.unpin_position()

    def dump(self, fname):
        """
        Writes the cloth to file.
        """
        write_to_file(self, fname)

    def reset(self):
        """
        Resets cloth to its initial state.
        """
        self.mouse.reset()
        width, height = self.initial_params[0]
        dx, dy = self.initial_params[1]
        gravity = self.initial_params[2]
        elasticity = self.initial_params[3]
        pin_cond = self.initial_params[4]
        self.pts = []
        self.tensioners = []
        for i in range(height):
            for j in range(width):
                pt = Point(self.mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity)
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pt.add_constraint(self.pts[-1])
                if pin_cond(j, i, height, width):
                    pt.pinned = True
                self.pts.append(pt)
