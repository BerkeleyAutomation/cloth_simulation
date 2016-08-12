from point import *
from mouse import *

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
        self.grabbed_pts = []
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