#!/usr/bin/env python2

from __future__ import division
import math
import numpy as np
from scipy.stats import beta
from matplotlib import pyplot as plt
import sys


# Part II
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
];
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
];
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
];
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
];
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
];
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
];

# convert to numpy arrays because I ain't working with lists of lists
exampleA1=np.array(exampleA1)
exampleA2=np.array(exampleA2)
exampleB1=np.array(exampleB1)
exampleB2=np.array(exampleB2)
exampleC1=np.array(exampleC1)
exampleC2=np.array(exampleC2)


velocities=[-2,-1,0,1,2]
priors=np.ones([3,5,5])
likelihoods=np.ones([3,5,5])
posteriors=np.zeros([3,5,5])
list_o_matricies=[[exampleA1,exampleA2],[exampleB1,exampleB2],[exampleC1,exampleC2]]
i=0
sigma=0.5
for set_of_examples in list_o_matricies:
    new_like=likelihoods[i]
    new_prior=priors[i]
    for vx in velocities:
        for vy in velocities[::-1]:
            new_prior[velocities.index(vy),velocities.index(vx)]=np.exp(-(pow(vx,2)+pow(vy,2))/(2*sigma))
            for x in range(set_of_examples[0].shape[0]):
                for y in range(set_of_examples[0].shape[1]):
                    if ((x+vx)>9) or ((x+vx)<0) or ((y+vy)>9) or ((y+vy)<0):
                        continue
                    else:
                        new_like[velocities.index(vy),velocities.index(vx)]=new_like[velocities.index(vy),velocities.index(vx)]*np.exp(-pow(set_of_examples[0][y,x]-set_of_examples[1][y+vy,x+vx],2)/(2*sigma))
    #  y=x[::-1]
    likelihoods[i]=new_like
    print likelihoods
    sys.exit()
    priors[i]=new_prior
    posteriors[i]=np.log(new_like)+np.log(new_prior)
    i=i+1


x=range(-2,3)
# task 1
for j in range(3):
    plt.clf()
    cs=plt.contourf(x,x,np.log(likelihoods[j]),corner_mask=False)
    if j==0:
        letter='A'
    elif j==1:
        letter='B'
    elif j==2:
        letter='C'
    plt.colorbar(cs,ticks=cs.levels)
    plt.xlabel('X velocity')
    plt.ylabel('Y velocity')
    plt.title('Log Likelihood of Motion (unscaled) | Example '+letter)
    plt.show()
# task 2
for j in range(3):
    plt.clf()
    cs=plt.contourf(x,x,posteriors[j],corner_mask=False)
    if j==0:
        letter='A'
    elif j==1:
        letter='B'
    elif j==2:
        letter='C'
    plt.colorbar(cs,ticks=cs.levels)
    plt.xlabel('X velocity')
    plt.ylabel('Y velocity')
    plt.title('Log Posterior of Motion (unscaled) | Example '+letter)
    plt.show()
# task 3
for j in range(3):
    posteriors[j]=np.exp(posteriors[j])
    posteriors[j]=np.divide(posteriors[j],np.sum(posteriors[j]))
    plt.clf()
    cs=plt.contourf(x,x,posteriors[j],corner_mask=False)
    if j==0:
        letter='A'
    elif j==1:
        letter='B'
    elif j==2:
        letter='C'
    plt.colorbar(cs,ticks=cs.levels)
    plt.xlabel('X velocity')
    plt.ylabel('Y velocity')
    plt.title('Posterior of Motion (scaled) | Example '+letter)
    plt.show()

# Part III

def partIII(priors,obs,sigma,prior_title):
    post=[0.0,0.0,0.0,0.0]
    directions=[[-1,0],[0,1],[1,0],[0,-1]]
    for direction in directions: # left[dx,dy], up, right, down
        pRx=1/(math.sqrt(2*math.pi*sigma**2))*np.exp(-((obs[0]-direction[0])**2)/(2*sigma**2))
        pRy=1/(math.sqrt(2*math.pi*sigma**2))*np.exp(-((obs[1]-direction[1])**2)/(2*sigma**2))
        pBx=1/(math.sqrt(2*math.pi*sigma**2))*np.exp(-((obs[2]-direction[0])**2)/(2*sigma**2))
        pBy=1/(math.sqrt(2*math.pi*sigma**2))*np.exp(-((obs[3]-direction[1])**2)/(2*sigma**2))
        post[directions.index(direction)]=priors[directions.index(direction)]*pRx*pRy*pBx*pBy
    post=post/sum(post)
    plt.clf()
    plt.bar(['left','up','right','down'],post)
    plt.xlabel('Direcition of Motion')
    plt.ylabel('Normalized Probability')
    plt.title('P(Direction|Rx,Ry,Bx,By) | $\sigma$='+str(sigma)+' | '+prior_title)
    plt.show()

# task 1
priors=[1/4,1/4,1/4,1/4]
obs=[0.75,-0.6,1.4,-0.2]
partIII(priors,obs,1,'Uniform Priors')
# task 2
partIII(priors,obs,5,'Uniform Priors')
priors=[1/8,1/8,1/8,5/8]
# task 3
partIII(priors,obs,1,'Large Down Prior')
# task 4
partIII(priors,obs,5,'Large Down Prior')
