import numpy as np
import ipdb, IPython
import pickle, sys, os
import matplotlib.pyplot as plt

if __name__ == '__main__':
	files = ["blobpolicies/data/" + file for file in os.listdir("blobpolicies/data")]
	scores = []
	for file in files:
		data = pickle.load(open(file, "rb"))
		last = data[-1]
		score = last[0]
		print score
		scores.append(score)
	plt.plot(scores)
	plt.show()
	pickle.dump(scores, open("blobpolicies/results.p", "w+"))
