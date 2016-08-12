from point import *
from cloth import *
from mouse import *

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