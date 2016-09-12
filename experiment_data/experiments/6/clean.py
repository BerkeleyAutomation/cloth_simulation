import matplotlib.pyplot as plt
import numpy as np
import sys, pickle, os, copy, IPython

if __name__ == '__main__':
    writefiles = ["nopolicydata.p", "nograspdata.p", "policy1data.p", "policy2data.p"]
    j = 0
    for file in ["nopolicy.p", "nograb.p", "test1.p", "test2.p"]:
        print file
        data = pickle.load(open(file, "rb"))
        lst1 = []
        d = {}
        # IPython.embed()
        for i in range(21):
            gravity = -i * 200
            lst = [a[-2] for a in data[gravity]]
            lst1.extend(lst)
            d[gravity] = lst
        real_scores = lst1
        plt.plot(real_scores)
        plt.show()
        pickle.dump(d, open("data/"+writefiles[j], "w+"))
        j += 1

