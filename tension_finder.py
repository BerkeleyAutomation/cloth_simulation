import numpy as np
import matplotlib.pyplot as plt
import sys, pickle, os
from mouse import *
from shapecloth import *
from cloth import *


class TensionPointFinder(object):

    def __init__(self, cloth):
        self.cloth = cloth

    def find_valid_pts(self):
        width, height = self.cloth.initial_params[0]
        dx, dy = self.cloth.initial_params[1]
        shape_fn = self.cloth.initial_params[2]
        grid = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                if shape_fn(j * dy + 50, i * dx + 50):
                    grid[i, j] = 1
        lock = False
        continued = False
        for i in range(height):
            lock = False
            for j in range(width):
                if lock:
                    grid[i, j] = 1
                elif grid[i, j]:
                    lock = True
        return -grid + 1

if __name__ == '__main__':
    shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
    mouse = Mouse(down=True)
    cloth = ShapeCloth(shape_fn, mouse)
    tpf = TensionPointFinder(cloth)
    plt.imshow(tpf.find_valid_pts())
    plt.show()
