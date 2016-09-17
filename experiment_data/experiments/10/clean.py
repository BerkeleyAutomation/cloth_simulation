import numpy as np
import ipdb, IPython
import pickle, sys, os
import matplotlib.pyplot as plt
import IPython

if __name__ == '__main__':
	files = "test9_1.p", "test9_2.p", "test9_3.p", "test9_4.p", "test9_5.p"
	# files = "test3_1.p", "test3_2.p", "test3_3.p", "test3_4.p", "test3_5.p"
	dic = {}
	keys = [625, 100, 400, 25, 2500]
	for i in range(len(files)):
		file = files[i]
		data = pickle.load(open("blob/" + file, "rb"))
		d = [d[0] for d in data]
		print d, np.mean(d)
		print i
		dic[keys[i]] = d
	pickle.dump(dic, open("resolution900.p", "w+"))