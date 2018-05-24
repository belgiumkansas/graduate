#!/usr/bin/env python2

import math
import matplotlib.pyplot as plt
import numpy as np


expected_prior = lambda s, sigma: math.exp(-(4*(s/sigma)))

def posterior_likelihood(sigma, obsvs):
    hyp_p = []
    H = [0] * 10
    H_X = []
    for i in range(1, 11):
        hyp_p.append(i)
        for obsv in obsvs:
            if i >= abs(max(obsv, key=abs)):
                H[i-1] += 1

        H_X.append(H[i-1]*expected_prior(i, sigma))

    H_X = [h/sum(H_X) for h in H_X]
    return H_X, hyp_p





def bar_graph(size, P, sigma, task):
    plt.bar(size, P)
    plt.ylabel('p(s)')
    plt.xlabel('s')
    if task == 1:
        plt.title('Expected Size Prior $\sigma$ = %d' % sigma)
    elif task == 2:
        plt.title('P(H|X) $\sigma$ = %d' % sigma)
    plt.show()


def task1(graphit=False, printit=False ):
    P = []
    size = []

    sigma = 6.0
    for i in range(0, 10):
        size.append(i)
        P.append(expected_prior(i, sigma))
    if printit: print P
    P = [p/sum(P) for p in P]
    if graphit: bar_graph(size, P, sigma, 1)

    sigma = 12.0
    for i in range(0,10):
        size.append(i)
        P.append(expected_prior(i, sigma))
    if printit: print P
    P = [p/sum(P) for p in P]
    if graphit: bar_graph(size, P, sigma, 1)

def task2(sigma, graphit=False, printit=False):
    observations = [(-1.5, .5)]

    H_X, X = posterior_likelihood(sigma, observations)
    if graphit: bar_graph(X, H_X, sigma, 2)

    '''for i in range(0,21, 2):
        Hypothesis.append(i)
        if (i) >= box_location:
            H_X.append(expected_prior(i, sigma))
        else:
            H_X.append(0)
    H_X = [h/sum(H_X) for h in H_X]

    if printit: print H_X
    if graphit: bar_graph(Hypothesis, H_X, sigma, 2)'''

def task3(H_X, graphit=False, printit=False):
    pass




def main():
    #task1()
    #task2(12.0)
    task2(10.0, graphit=True)







if __name__ == '__main__':
    main()
