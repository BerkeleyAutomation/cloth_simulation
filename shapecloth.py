from point import *
from cloth import *
from mouse import *

"""
A subclass of cloth, on which a shape pattern is drawn. It also can be grabbed and tensioned.
"""
class ShapeCloth(Cloth):

    def __init__(self, mouse, shape_fn, width=50, height=50, dx=10, dy=10,gravity=-1000.0, elasticity=1.0, pin_cond="default"):
        """
        A cloth on which a shape can be drawn. It can also be grabbed and tensioned at specific coordinates. It takes in a function shape_fn that takes in 2 arguments, x and y, that specify whether or not a point is located on the outline of a shape.
        """
        self.pts = []
        self.shapepts = []
        self.normalpts = []
        self.tensioners = []
        self.mouse = mouse
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
                if shape_fn(pt.x, pt.y):
                    self.shapepts.append(pt)
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
                if pt in self.shapepts:
                    self.shapepts.remove(pt)
                else:
                    self.normalpts.remove(pt)

