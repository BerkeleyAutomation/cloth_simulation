from point import *
from cloth import *
from mouse import *
from registration import *

"""
A subclass of cloth, on which a shape pattern is drawn. It also can be grabbed and tensioned.
"""
class ShapeCloth(Cloth):

    def __init__(self, shape_fn, mouse=None, width=50, height=50, dx=10, dy=10,gravity=-2500.0, elasticity=1.0, pin_cond="default", bounds=(600, 600, 800), blobs=None, corners=None):
        """
        A cloth on which a shape can be drawn. It can also be grabbed and tensioned at specific coordinates. It takes in a function shape_fn that takes in 2 arguments, x and y, that specify whether or not a point is located on the outline of a shape.
        """
        if not mouse:
            mouse = Mouse(bounds=bounds)
        self.pts = []
        self.shapepts = []
        self.normalpts = []
        self.tensioners = []
        self.bounds = bounds
        self.mouse = mouse
        self.allpts = {}
        self.blobs = []
        self.blobpts = []
        if self.blobs != None and corners != None:
            self.blob_fn = get_blob_fn(corners, blobs)
            for blob in blobs:
                self.blobs.append([])
        else:
            self.blob_fn = lambda x, y: False
        if pin_cond == "default":
            pin_cond = lambda x, y, height, width: y == height - 1 or y == 0
        for i in range(height):
            for j in range(width):
                pt = Point(mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity, bounds=bounds, identity=j + i * height)
                self.allpts[i * height + j] = pt
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pt.add_constraint(self.pts[-1])
                if pin_cond(j, i, height, width):
                    pt.pinned = True
                if shape_fn(pt.x, pt.y):
                    self.shapepts.append(pt)
                    pt.shape = 1
                else:
                    self.normalpts.append(pt)
                self.pts.append(pt)
                b = self.blob_fn(pt.x, pt.y)
                if b != -1 and b != False:
                    self.blobs[b].append(pt)
                    if pt in self.shapepts:
                        self.shapepts.remove(pt)
                    if pt in self.normalpts:
                        self.normalpts.remove(pt)
                    self.blobpts.append(pt)
        self.pts, self.normalpts, self.shapepts = set(self.pts), set(self.normalpts), set(self.shapepts)
        self.initial_params = [(width, height), (dx, dy), shape_fn, gravity, elasticity, pin_cond]


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
        toremoveshape, toremovenorm, toremoveblob = [], [], []
        for pt in self.pts:
            if pt.constraints == []:
                if pt in self.shapepts:
                    toremoveshape.append(pt)
                elif pt in self.normalpts:
                    toremovenorm.append(pt)
                else:
                    toremoveblob.append(pt)

        for pt in toremovenorm:
            self.pts.remove(pt)
            self.normalpts.remove(pt)
        for pt in toremoveshape:
            self.pts.remove(pt)
            self.shapepts.remove(pt)
        for pt in toremoveblob:
            self.pts.remove(pt)
            self.blobpts.remove(pt)
            for blob in self.blobs:
                if pt in blob:
                    blob.remove(pt)
        return len(toremoveshape)

    def reset(self):
        """
        Resets cloth to its initial state.
        """
        self.mouse.reset()
        width, height = self.initial_params[0]
        dx, dy = self.initial_params[1]
        shape_fn = self.initial_params[2]
        gravity = self.initial_params[3]
        elasticity = self.initial_params[4]
        pin_cond = self.initial_params[5]
        self.pts = []
        self.shapepts = []
        self.normalpts = []
        self.tensioners = []
        self.allpts = {}
        self.blobpts = []
        numblobs = len(self.blobs)
        self.blobs = []
        for i in range(numblobs):
            self.blobs.append([])
        for i in range(height):
            for j in range(width):
                pt = Point(self.mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity, identity=j + i * height)
                self.allpts[i * height + j] = pt
                if i > 0:
                    pt.add_constraint(self.pts[width * (i - 1) + j])
                if j > 0:
                    pt.add_constraint(self.pts[-1])
                if pin_cond(j, i, height, width):
                    pt.pinned = True
                if shape_fn(pt.x, pt.y):
                    self.shapepts.append(pt)
                    pt.shape = 1
                else:
                    self.normalpts.append(pt)
                self.pts.append(pt)
                b = self.blob_fn(pt.x, pt.y)
                if b != -1 and b != False:
                    self.blobs[b].append(pt)
                    if pt in self.shapepts:
                        self.shapepts.remove(pt)
                    if pt in self.normalpts:
                        self.normalpts.remove(pt)
                    self.blobpts.append(pt)
        self.pts = set(self.pts)

    @property
    def centroids(self):
        centroids = []
        for blob in self.blobs:
            if len(blob):
                centroids.append(self.centroidnp(np.array([[pt.x, pt.y, pt.z] for pt in blob])))
            else:
                centroids.append((-1, -1, -1))
        return centroids
    
    def centroidnp(self, arr):
        length = arr.shape[0]
        sum_x = np.sum(arr[:, 0])
        sum_y = np.sum(arr[:, 1])
        sum_z = np.sum(arr[:, 2])
        return sum_x/length, sum_y/length, sum_z/length

    def find_closest_shapept(self, x, y):
        pt = self.shapepts[np.argmin(np.linalg.norm(np.array([(pt.x, pt.y) for pt in self.shapepts]) - np.tile(np.array((x, y)), (len(self.shapepts), 1)),  axis=1))]
        return pt.x, pt.y

    def find_dtheta(self, x0, y0, x1, y1, x2, y2):
        v1 = np.array((x1, y1)) - np.array((x0, y0)) + 1e-5
        v2 = np.array((x2, y2)) - np.array((x0, y0)) + 1e-5
        v1, v2 = v1 / np.linalg.norm(v1), v2 / np.linalg.norm(v2)
        costheta = np.dot(v1, v2)
        theta = np.arccos(costheta)
        return theta


