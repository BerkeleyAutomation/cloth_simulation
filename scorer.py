import numpy as np
import matplotlib.pyplot as plt
import sys, os, pickle
from simulation import *
from shapecloth import *
from mouse import *

class Scorer(object):

	def __init__(self, fn=0):
		if not fn:
			self.score_fn = lambda c: len(c.shapepts)
		elif fn == 1:
			self.score_fn = bad_edge_score
		else:
			self.score_fn = fn

	def score(self, cloth):
		return self.score_fn(cloth)

def bad_edge_score(cloth):
	count = 0
	for pt in cloth.normalpts:
		for constraint in pt.constraints:
			if (constraint.p1 in cloth.normalpts and constraint.p2 in cloth.shapepts) or (constraint.p1 in cloth.shapepts and constraint.p2 in cloth.normalpts):
				count += 1
	return count

