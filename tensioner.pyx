import numpy as np
from cloth import *

"""
A Tensioner grabs a circular region of cloth and fixes it in place. It can then tug on the cloth in a particular direction.
"""

class Tensioner(object):

    def __init__(self, x, y, cloth, max_displacement=False):
        self.x, self.y = x, y
        self.origx, self.origy = x, y
        self.dz = 0
        self.cloth = cloth
        self.grabbed_pts = []
        cloth.add_tensioner(self)
        self.pin_position(x, y)
        if max_displacement:
            self.max_displacement = max_displacement
        else:
            self.max_displacement = float('inf')

    def pin_position(self, x, y):
        """
        Grab a position on the cloth and pin it in place.
        """
        for pt in self.cloth.pts:
            if abs((pt.x - x) ** 2 + (pt.y - y) ** 2) < 1000:
                pt.pinned = True
                self.grabbed_pts.append(pt)

    def unpin_position(self):
        """
        Let go of a grabbed position.
        """
        for pt in self.grabbed_pts:
            pt.pinned = False
        self.grabbed_pts = []
        self.cloth.remove(self)

    def tension(self, x, y, z=0):
        """
        Tug on the grabbed area in a direction.
        """
        if np.linalg.norm([self.x - self.origx + x, self.y - self.origy + y, self.dz + z]) > self.max_displacement:
            return
        for pt in self.grabbed_pts:
            pt.px, pt.py, pt.pz = pt.x, pt.y, pt.z
            pt.x, pt.y, pt.z = x + pt.x, y + pt.y, z + pt.z
            self.dz += z
        self.x += x
        self.y += y

    @property
    def displacement(self):
        """
        Displacement from the original grabbing position
        """
        return np.array((self.x - self.origx, self.y - self.origy, self.dz))
    