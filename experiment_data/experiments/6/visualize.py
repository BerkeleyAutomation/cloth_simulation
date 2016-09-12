import matplotlib.pyplot as plt
import numpy as np
import sys, pickle, os, copy

if __name__ == '__main__':
    if sys.argv[1] == "training":
        data = sys.argv[2]
        with open(data, "rb") as f:
            data = pickle.load(f)
        real_scores = [a[0] for a in data]
        policy_scores = np.array([sum(a[1]) for a in data])
        plt.plot(real_scores)
        plt.show()

        plt.plot(policy_scores)
        plt.show()
    else:
        if "nodict" in sys.argv:
            data = sys.argv[1]
            with open(data, "rb") as f:
                data = pickle.load(f)
            real_scores = [a[1] for a in data]
            policy_scores = np.array([sum(a[2]) for a in data])
            plt.plot(real_scores)
            plt.show()
            plt.plot(policy_scores)
            plt.show()      
        else:
            data = sys.argv[1]
            with open(data, "rb") as f:
                data = pickle.load(f)
            lst1, lst2 = [], []
            for i in range(21):
                gravity = -i * 200
                lst1.extend([a[-2] for a in data[gravity]])
                lst2.extend([a[-1] for a in data[gravity]])
            real_scores = lst1
            policy_scores = lst2
            plt.plot(real_scores)
            plt.show()
            plt.plot(policy_scores)
            plt.show()  