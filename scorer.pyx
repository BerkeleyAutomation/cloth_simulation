import numpy as np
import matplotlib.pyplot as plt
import sys, os, pickle
from simulation import *
from shapecloth import *
from mouse import *

"""
A scorer object that assigns a score to the state of a cloth object. The scoring function can be overridden or can be specified from a selection of previously defined functions.
"""

class Scorer(object):

	def __init__(self, fn=0):
		if fn == 0:
			self.score_fn = lambda c: -len(c.shapepts)
		elif fn == 1:
			self.score_fn = bad_edge_score
		else:
			self.score_fn = fn

	def score(self, cloth):
		return self.score_fn(cloth)

def bad_edge_score(cloth):
	"""
	Another naive scoring function that counts the number of edges from shapepts to normalpts.
	"""
	count = 0
	for pt in cloth.normalpts:
		for constraint in pt.constraints:
			if (constraint.p1 in cloth.normalpts and constraint.p2 in cloth.shapepts) or (constraint.p1 in cloth.shapepts and constraint.p2 in cloth.normalpts):
				count += 1
	return -count

