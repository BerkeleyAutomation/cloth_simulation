from math import sqrt
from constraint import *
from mouse import *
import numpy as np

"""
A class that simulates a point mass. A cloth is made up of a collection of these interacting with each other.
"""
class Point(object):

    def __init__(self, mouse, x=0, y=0, z=0, gravity=-1000.0, elasticity=1.0, bounds=(600, 600, 800), shape=0, identity=-1, noise=0):
        """
        Initializes an instance of a particle.
        """
        self.mouse = mouse
        self.x, self.y, self.z = x, y, z
        self.px, self.py, self.pz = x, y, z
        self.vx, self.vy, self.vz = 0, 0, 0
        self.constraints = []
        self.pinned = False
        self.gravity = gravity
        self.elasticity = elasticity
        self.bounds = bounds
        self.shape = shape
        self.noise = noise
        self.identity = identity

    def add_constraint(self, pt):
        """
        Adds a constraint between this point and another point.
        """
        self.constraints.append(Constraint(self, pt, elasticity=self.elasticity))

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
        boundsx, boundsy, boundsz = self.bounds
        if self.x >= boundsx:
            self.x = 2 * boundsx - self.x + np.random.randn() * self.noise
        elif self.x < 1:
            self.x = 2 - self.x + np.random.randn() * self.noise
        if self.y >= boundsy:
            self.y = 2 * boundsy - self.y + np.random.randn() * self.noise
        elif self.y < 1:
            self.y = 2 - self.y + np.random.randn() * self.noise
        if self.z >= boundsz:
            self.z = 2 * boundsz - self.z + np.random.randn() * self.noise
        elif self.z <= -boundsz:
            self.z = -2 * boundsz - self.z + np.random.randn() * self.noise

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

        self.add_force(0, 0, self.gravity)
        delta *= delta

        nx = self.x + ((self.x - self.px)) * 0.99 + ((self.vx / 2.0) * delta) + np.random.randn() * self.noise
        ny = self.y + ((self.y - self.py)) * 0.99 + ((self.vy / 2.0) * delta) + np.random.randn() * self.noise
        nz = self.z + ((self.vz / 2.0) * delta) + np.random.randn() * self.noise

        self.px, self.py, self.pz = self.x, self.y, self.z
        self.x, self.y, self.z = nx, ny, nz
        self.vx, self.vy, self.vz = 0, 0, 0
        if self.noise:
            dx, dy, dz = np.random.randn() * self.noise, np.random.randn() * self.noise, np.random.randn() * self.noise
            self.x += dx
            self.y += dy
            self.z += dz