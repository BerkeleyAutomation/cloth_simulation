from point import *
from cloth import *
from mouse import *

"""
A subclass of cloth, on which a circle pattern is drawn. It also can be grabbed and tensioned.
"""
class CircleCloth(Cloth):

    def __init__(self, mouse, width=50, height=50, dx=10, dy=10, centerx=300, centery=300, radius=150, gravity=-1000.0, elasticity=1.0, pin_cond=default_pin_condition):
        """
        A cloth on which a circle can be drawn. It can also be grabbed and tensioned at specific coordinates.
        """
        self.pts = []
        self.circlepts = []
        self.normalpts = []
        self.tensioners = []
        self.mouse = mouse
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

def default_pin_condition(x, y, height, width):
    return y == height - 1 or y == 0