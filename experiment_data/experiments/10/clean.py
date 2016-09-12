import numpy as np
import ipdb, IPython
import pickle, sys, os
import matplotlib.pyplot as plt
import IPython

if __name__ == '__main__':
	files = "test5_1.p", "test5_2.p", "test5_3.p", "test5_4.p", "test5_5.p", "test1.p", "test2.p", "test3.p", "test4.p"
	for file in files:
		data = pickle.load(open("circle/" + file, "rb"))
		d = [d[0] for d in data]
		pickle.dump(d, open("circle/data/" + file, "w+"))

