#!/usr/bin/env python2

import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import gamma
from scipy.stats import norm
import matplotlib.mlab as mlab




university = {0: 'metro', 1: 'cu'}
major = {0: 'buisness', 1: 'compsi'}

def task1():
    salary = 60
    N = 100000
    samples = []
    for i in range(N):

        intlgnc = np.random.normal(100, 15)
        mjr =  np.random.randn() < 1/(1+math.exp(-(intlgnc-110)/5))
        unv = np.random.randn() < 1/(1+math.exp(-(intlgnc-100)/5))
        a = .1*intlgnc + mjr + 3* unv
        weight = gamma.cdf(salary, a, scale=5)

        samples.append((unv, mjr, weight))

    cu_cs = 0
    cu_bs = 0
    ms_cs = 0
    ms_bs = 0
    for sample in samples:
        #CU and CS
        if sample[0:2] == (True, True): cu_cs += sample[2]
        #CU and buisness
        elif sample[0:2] == (True, False): cu_bs += sample[2]
        #MS and CS
        elif sample[0:2] == (False, True): ms_cs += sample[2]
        #MS and BS
        elif sample[0:2] == (False, False): ms_bs += sample[2]
        else: print "error"

    total_sum = cu_cs + cu_bs + ms_cs + ms_bs

    print cu_cs/total_sum
    print cu_bs/total_sum
    print ms_cs/total_sum
    print ms_bs/total_sum


def cnd_smpl(x, y, yval, mu, sig):
    x = x-1
    y = y -1
    mucnd = mu[x] + sig[x][y]*(1/sig[y][y])*(yval - mu[y])
    sigcnd = sig[x][x] - sig[x][y]*(1/sig[y][y])*sig[y][x]
    return np.random.normal(mucnd, sigcnd)

def task2():
    fig, (ax1, ax2) = plt.subplots(1,2)
    mu = (1, 0)
    sig = ((1, -.5), (-.5, 3))
    N = 100000
    x = np.linspace(-10, 10, 120)
    x_init = (0 , 0)

    samples_x1 = [1]
    samples_x2 = [0]
    for num in range(N):
        px1_x2 = cnd_smpl(1, 2, samples_x2[num], mu, sig)
        px2_x1 = cnd_smpl(2, 1, samples_x1[num], mu, sig)
        samples_x1.append(px1_x2)
        samples_x2.append(px2_x1)

    ax1.hist(samples_x1, 60, normed=True)
    ax1.plot(x, norm(mu[0], sig[0][0]).pdf(x))
    ax2.hist(samples_x2, 60, normed=True)
    ax2.plot(x, norm(mu[1], sig[1][1]).pdf(x))
    plt.show()


def q(x, y):
    g1 = mlab.bivariate_normal(x, y, 1.0, 1.0, -1, -1, -0.8)
    g2 = mlab.bivariate_normal(x, y, 1.5, 0.8, 1, 2, 0.6)
    return 0.6*g1+28.4*g2/(0.6+28.4)

def task3():
    '''Metropolis Hastings'''
    '''http://isaacslavitt.com/2013/12/30/metropolis-hastings-and-slice-sampling/'''
    N = 100000
    s = 10
    r = np.zeros(2)
    p = q(r[0], r[1])
    print p
    samples = []
    for i in xrange(N):
        rn = r + np.random.normal(size=2)
        pn = q(rn[0], rn[1])
        if pn >= p:
            p = pn
            r = rn
        else:
            u = np.random.rand()
            if u < pn/p:
                p = pn
                r = rn
        if i % s == 0:
            samples.append(r)

    samples = np.array(samples)
    plt.scatter(samples[:, 0], samples[:, 1], alpha=0.5, s=1)

    '''Plot target'''
    dx = 0.01
    x = np.arange(np.min(samples), np.max(samples), dx)
    y = np.arange(np.min(samples), np.max(samples), dx)
    X, Y = np.meshgrid(x, y)
    Z = q(X, Y)
    CS = plt.contour(X, Y, Z, 10)
    plt.clabel(CS, inline=1, fontsize=10)
    plt.show()



if __name__ == '__main__':
    task3()
