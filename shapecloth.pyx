from point import *
from cloth import *
from mouse import *
from registration import *
from collections import deque
from scipy import signal
from scipy import stats
import matplotlib.pyplot as plt
import IPython


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
                pt = Point(mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity, bounds=bounds, identity=j + i * width)
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
        self.setup()


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
                pt = Point(self.mouse, 50 + dx * j, 50 + dy * i, gravity=gravity, elasticity=elasticity, identity=j + i * width)
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


    def setup(self, plot=False):
        width, height = self.initial_params[0]
        dx, dy = self.initial_params[1]
        shape_fn = self.initial_params[2]
        grid = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                if shape_fn(j * dy + 50, i * dx + 50):
                    grid[i, j] = 1
        grid = signal.convolve2d(grid, np.ones((1, 1)), mode='same')
        grid = stats.threshold(grid, threshmax=1e-10, newval=1)
        if plot:
            plt.imshow(np.flipud(grid), cmap='Greys_r')
            plt.show()
        self.shape_area = np.sum(grid)
        grid = -grid + 1
        grid2 = np.zeros_like(grid)
        queue = deque([])
        seen = []
        queue.append((0,0))
        while len(queue) > 0:
            pos = queue.popleft()
            if pos in seen:
                continue
            seen.append(pos)
            if grid[pos]:
                grid2[pos] = 1
                neighbors = [(pos[0] + 1, pos[1]), (pos[0] - 1, pos[1]), (pos[0], pos[1] - 1), (pos[0], pos[1] + 1)]
                for pos in neighbors:
                    if pos in seen or min(pos) < 0 or pos[0] >= height or pos[1] >= width:
                        pass
                    else:
                        queue.append(pos)
        if plot:
            plt.imshow(np.flipud(grid2), cmap='Greys_r')
            plt.show()
        self.outgrid = grid2
        self.shapegrid = -grid + 1
        self.out_area = np.sum(self.outgrid)
        self.in_area = height * width - self.out_area - self.shape_area
        return grid2

    def setup_helper(self, plot=False):
        width, height = self.initial_params[0]
        dx, dy = self.initial_params[1]
        shape_fn = self.initial_params[2]
        grid = np.zeros((height, width))
        for key in self.allpts.keys():
            pt = self.allpts[key]
            if pt in self.pts:
                continue
            else:
                pos = (np.floor(pt.identity / width), pt.identity % width)
                grid[pos] = 1
        grid = signal.convolve2d(grid, np.ones((1, 1)), mode='same')
        grid = stats.threshold(grid, threshmax=1e-10, newval=1)
        grid = grid + self.shapegrid
        grid = stats.threshold(grid, threshmax=1e-10, newval=1)
        if plot:
            plt.imshow(np.flipud(grid), cmap='Greys_r')
            plt.show()
        grid = -grid + 1
        grid2 = np.zeros_like(grid)
        queue = deque([])
        seen = []
        queue.append((10,0))
        grid = stats.threshold(grid, threshmax=1e-10, newval=1)
        if plot:
            plt.imshow(np.flipud(grid), cmap='Greys_r')
            plt.show()
        while len(queue) > 0:
            pos = queue.popleft()
            if pos in seen:
                continue
            seen.append(pos)
            if grid[pos]:
                grid2[pos] = 1
                neighbors = [(pos[0] + 1, pos[1]), (pos[0] - 1, pos[1]), (pos[0], pos[1] - 1), (pos[0], pos[1] + 1)]
                for pos in neighbors:
                    if pos in seen or min(pos) < 0 or pos[0] >= height or pos[1] >= width:
                        pass
                    else:
                        queue.append(pos)
        if plot:
            plt.imshow(np.flipud(grid2), cmap='Greys_r')
            plt.show()
        newoutarea = np.sum(grid2)

        grid2 = np.zeros_like(grid)
        queue = deque([])
        seen = []
        queue.append((25,25))
        grid = stats.threshold(grid, threshmax=1e-10, newval=1)
        if plot:
            plt.imshow(np.flipud(grid), cmap='Greys_r')
            plt.show()
        while len(queue) > 0:
            pos = queue.popleft()
            if pos in seen:
                continue
            seen.append(pos)
            if grid[pos]:
                grid2[pos] = 1
                neighbors = [(pos[0] + 1, pos[1]), (pos[0] - 1, pos[1]), (pos[0], pos[1] - 1), (pos[0], pos[1] + 1)]
                for pos in neighbors:
                    if pos in seen or min(pos) < 0 or pos[0] >= height or pos[1] >= width:
                        pass
                    else:
                        queue.append(pos)
        if plot:
            plt.imshow(np.flipud(grid2), cmap='Greys_r')
            plt.show()

        newinarea = np.sum(grid2)
        din = self.in_area - newinarea
        dout = self.out_area - newoutarea
        score = din + dout
        print score, "Score"
        print din, dout
        print self.in_area, newinarea, self.out_area, newoutarea
        return score

    def evaluate(self, log=False):
        return = self.setup_helper()






