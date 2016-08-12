from point import *
from mouse import *
from tensioner import *

"""
A cloth class, which consists of a collection of points and their corresponding constraints.
"""
class Cloth(object):

    def __init__(self, mouse, width=50, height=50, dx=10, dy=10, gravity=-1000.0, elasticity=1.0, pin_cond=False):
        """
        Creates a cloth with width x height points spaced dx and dy apart. The top and bottom row of points are pinned in place.
        """
        self.mouse = mouse
        self.pts = []
        self.tensioners = []
        if pin_cond == "default":
            pin_cond = lambda x, y, height, width: y == height - 1 or y == 0
        for i in range(height):
            for j in range(width):
                pt = Point(mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity)
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pass
                    pt.add_constraint(self.pts[-1])
                if pin_cond(j, i, height, width):
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
