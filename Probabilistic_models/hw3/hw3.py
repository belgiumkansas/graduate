#!/usr/bin/env python2
from __future__ import division
import numpy as np

import math

from scipy.stats import beta
import matplotlib.pyplot as plt
from scipy.ndimage.interpolation import shift
from scipy.linalg import expm


def weissPatterns():
    exampleA1 = [
        [1,0,0,0,0,0,0,0,0,0],
        [1,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
    ]
    exampleA2 = [
        [0,1,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
    ]
    exampleB1 = [
        [1,0,0,0,0,0,0,0,0,0],
        [1,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
    ]
    exampleB2 = [
        [1,0,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
    ]
    exampleC1 = [
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0],
    ]
    exampleC2 = [
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1,0,0],
        [0,0,0,0,0,0,0,1,0,0],
        [0,0,0,0,0,0,0,1,0,0],
        [0,0,0,0,0,0,0,1,0,0],
        [0,0,0,0,0,0,0,1,0,0],
        [0,0,0,0,0,0,0,1,0,0],
    ]
    exampleA1 = np.array(exampleA1)
    exampleA2 = np.array(exampleA2)
    exampleB1 = np.array(exampleB1)
    exampleB2 = np.array(exampleB2)
    exampleC1 = np.array(exampleC1)
    exampleC2 = np.array(exampleC2)
    return exampleA1,exampleA2, exampleB1, exampleB2, exampleC1, exampleC2


def part1():

    f, axs = plt.subplots(2, 4)
    outcome = [1, 0, 0, 1, 0, 0, 0]
    a, b = 1, 1
    x = np.linspace(0.0, 1.0, 101)
    axs[0, 0].plot(x, beta.pdf(x, a, b), lw=1, alpha=0.6)
    axs[0, 0].set_ylim(0, 3)
    axs[0, 0].set_title("prior")
    axs[0, 0].set_ylabel("Prob")
    axs[0, 0].set_xlabel("Bias")
    row, column = 0, 1
    trial = 0
    for flip in outcome:
        if flip == 1:
            a += 1
            title_outcome = 'H'
        else:
            b += 1
            title_outcome = 'T'
        trial += 1
        axs[row, column].plot(x, beta.pdf(x, a, b), lw=1, alpha=0.6)
        axs[row, column].set_ylim(0, 3)
        axs[row, column].set_title("trial "+ str(trial)+ " : " + title_outcome)
        axs[row, column].set_ylabel("Prob")
        axs[row, column].set_xlabel("Bias")
        column += 1
        if column == 4:
            row += 1
            column = 0

    plt.show()


def part2():
    #perimitives
    A1, A2, B1, B2, C1, C2 = weissPatterns()
    pixel_sets = [[A1, A2], [B1, B2], [C1, C2]]
    vel = [-2, -1, 0, 1, 2]

    #small motion prior
    sigma = .5
    prior = np.ones([5, 5])
    for vx in vel:
        for vy in vel:
            prior[vx+2][vy+2] = np.exp(-(pow(vx, 2)+pow(vy, 2))/(2*sigma))

    #likelihood calculations and posteriors
    var = .5
    likelihood_array = np.zeros([3, 5, 5])
    index = 0
    posteriors = np.zeros([3, 5, 5])
    for pixel_set in pixel_sets:
        for vx in vel:
            for vy in vel:
                likelihood_calc = pixel_set[0] - shift(pixel_set[1], [-vx, -vy])
                likelihood_calc = likelihood_calc**2
                likelihood_calc = sum(sum(likelihood_calc))
                likelihood_calc = likelihood_calc/(-2.0*var)
                likelihood_calc = np.exp(likelihood_calc)
                likelihood_array[index][vx+2][vy+2] = likelihood_calc

        posteriors[index] = np.log(likelihood_array[index]) + np.log(prior)
        index += 1

    title = ['A', 'B', 'C']
    #task 1
    fig, axs = plt.subplots(1, 3)
    for i in range(3):
        img = axs[i].imshow(np.log(likelihood_array[i]), cmap="hot")
        #axs[i].set_xticklables(['-2', '-1', '0', '1', '2'])
        axs[i].set_title("image " + title[i])
        fig.colorbar(img, ax=axs[i])

    #task 2
    fig, axs = plt.subplots(1, 3)
    for i in range(3):
        img = axs[i].imshow(posteriors[i], cmap="hot")
        axs[i].set_title("image " + title[i])
        fig.colorbar(img, ax=axs[i])

    #task 3
    fig, axs = plt.subplots(1, 3)
    for i in range(3):
        posterior = np.exp(posteriors[i])
        print posterior
        scaled_posteriors = np.divide(posterior, np.sum(posterior))
        img = axs[i].imshow(posterior, cmap="hot")
        axs[i].set_title("image " + title[i])
        fig.colorbar(img, ax=axs[i])

    plt.show()
    return 0



def ambig_mm(priors, obsv, sigma, task):
    posterior = np.array([0.0, 0.0, 0.0, 0.0])
    index = 0
                 #up      down     left     right
    directions = [[0, 1], [0, -1], [-1, 0], [1, 0]]
    for direction in directions:
        Rx = np.exp(-(obsv[0]-direction[0])**2/(2*sigma**2))
        Rx = Rx * (1/math.sqrt(2*math.pi*sigma**2))

        Ry = np.exp(-(obsv[1]-direction[1])**2/(2*sigma**2))
        Ry = Ry * (1/math.sqrt(2*math.pi*sigma**2))

        Bx = np.exp(-(obsv[2]-direction[0])**2/(2*sigma**2))
        Bx = Bx * (1/math.sqrt(2*math.pi*sigma**2))

        By = np.exp(-(obsv[3]-direction[1])**2/(2*sigma**2))
        By = By * (1/math.sqrt(2*math.pi*sigma**2))

        posterior[index] = priors[index]*Rx*Ry*Bx*By
        index += 1

    posterior = posterior/sum(posterior)
    plt.bar(['up','down','left','right'],posterior)
    plt.xlabel('Direcition')
    plt.ylabel('Probability')
    plt.title("Task: " + task)
    plt.show()



def part3():
    #task 1
    priors = np.array([1/4, 1/4, 1/4, 1/4])
    obsv = np.array([3/4, -6/10, 14/10, -2/10])
    sigma = 1
    ambig_mm(priors, obsv, sigma, '1')

    #task 2
    sigma = 5
    ambig_mm(priors, obsv, sigma,'2')

    #task 3
    priors = np.array([1/8, 5/8, 1/8, 1/8])
    sigma = 1
    ambig_mm(priors, obsv, sigma, '3')

    #task 4
    sigma = 5
    ambig_mm(priors, obsv, sigma, '4')


if __name__ == '__main__':
    part3()
