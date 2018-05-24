
#!/usr/bin/env python2

from __future__ import division
import numpy as np
import sklearn
from random import random
import matplotlib.pyplot as plt


def crp(alpha=0.5, N=500):
    table_probs = []
    new_table_probs = []
    n = 1
    counts = [1]
    while n < N:
        assign_probs = [None] * (len(counts) + 1)
        for i in range(len(counts)):
            assign_probs[i] = counts[i] / (n - 1 + alpha)
        assign_probs[-1] = alpha /(n - 1 + alpha)
        table = draw_table(assign_probs)
        if table == len(counts):
            counts.append(1)
        else:
            counts[table] += 1
        new_table_probs.append(assign_probs[-1])
        n += 1

    return counts, new_table_probs

def draw_table(probs):
    norm = sum(probs)
    sample = random()
    total = 0.0
    for i in range(len(probs)):
        total += probs[i]/norm
        if sample < total:
            return i

def dpmm(alpha=0.5, N=500):
    table_probs = []
    n = 1
    counts = [1]
    x = np.random.uniform()
    y = np.random.uniform()
    theta_xy = [(np.random.normal(x,0.1), np.random.normal(y,0.1))]
    phi = [theta_xy[0]]
    table_list = [0]
    index=[0]

    while n < N:
        assign_probs = [None] * (len(counts) + 1)
        for i in range(len(counts)):
            assign_probs[i] = counts[i] / (n - 1 + alpha)
        assign_probs[-1] = alpha /(n - 1 + alpha)
        table = draw_table(assign_probs)
        if table >= len(counts):
            counts.append(1)
            x = np.random.uniform()
            y = np.random.uniform()
            theta_xy.append((x, y))
        else:
            counts[table] += 1
        g=(np.random.normal(theta_xy[table][0],0.1),np.random.normal(theta_xy[table][1],0.1))
        phi.append(g)
        table_list.append(table)
        n += 1

    return phi, table_list



if __name__ == '__main__':

    counts, new_table_probs = crp()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot([1] + new_table_probs)
    ax.set_title("Probability of Joining a New Table")
    ax.set_xlabel("People at Restaurant")
    ax.set_ylabel("Probability")
    #plt.show()



    phi, table = dpmm()
    x=[i[0] for i in phi]
    y=[i[1] for i in phi]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(x,y,c=table,marker='x')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("{} Clusters".format(max(table) + 1))
    plt.show()
