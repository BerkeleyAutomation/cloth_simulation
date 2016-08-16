import numpy as np
import sys, pickle
from math import sqrt
from shape_tracer import *
from scipy.interpolate import interp1d

def get_basis(corners):
    v1 = np.array(corners[1]) - np.array(corners[0])
    v2 = np.array(corners[2]) - np.array(corners[0])
    v3 = np.cross(v1, v2)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    v3 = v3 / np.linalg.norm(v3)
    return np.matrix(np.vstack((v1, v2, v3)).T)


def get_scale(corners, px_distance=500):
    return  float(px_distance) / np.linalg.norm(np.array(corners[0]) - np.array(corners[1]))

def transform_and_project_point(transform, scale, pt, corners, offset=(50,50)):
    return np.ravel(np.matrix(transform) * np.matrix((np.array(pt)  - np.array(corners[0]))).T * scale)[:2] + np.array(offset)

def get_shape_fn(corners, pts, interpolate=False):
    scale = get_scale(corners)
    basis = get_basis(corners)
    pxpts = []
    for pt in pts:
        pxpts.append(transform_and_project_point(basis, scale, pt, corners).tolist())
    if interpolate:
        pxpts = interpolation(np.array(pxpts), 10).tolist()
    return lambda x, y: np.min(np.linalg.norm(np.matrix(np.tile(np.array((x, y)), (len(pxpts), 1))) - pxpts, axis=1)) < 10

def interpolation(arr, factor):
    """
    Given a matrix of x,y coordinates, output a linearly interpolated matrix of coordinates with factor * arr.shape[0] points.
    """
    x = arr[:, 0]
    y = arr[:, 1]
    t = np.linspace(0,x.shape[0],num=x.shape[0])
    to_expand = [x, y]
    for i in range(len(to_expand)):
        spl = interp1d(t, np.ravel(to_expand[i]))
        to_expand[i] = spl(np.linspace(0,len(t), len(t)*factor))
    new_matrix = np.matrix(np.r_[0:len(t):1.0/factor])
    for i in to_expand:
        new_matrix = np.concatenate((new_matrix, np.matrix(i)), axis = 0)
    return new_matrix.T[:,1:]

if __name__ == "__main__":
    # corners = [(1, 1, 0), (2, 1, 0), (1, 2, 0), (2, 2, 0)]
    # fn = get_shape_fn(corners, [[2, 0, 0], [2,1,0], [2,2,0]])

    corners = load_robot_points()
    pts = load_robot_points("gauze_pts2.p")

    shape_fn = get_shape_fn(corners, pts, True)
