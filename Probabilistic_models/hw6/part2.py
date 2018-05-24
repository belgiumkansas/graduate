#!/usr/bin/env python2
from __future__ import division
import pymc3 as pm

def part2():

    with pm.Model() as model:
        G1 = pm.Bernoulli('G1', 0.5)
        ## p(G2|G1)
        G2_p = pm.Deterministic('G2_p', pm.math.switch(G1, 0.9, 0.1))
        G2 = pm.Bernoulli('G2', G2_p)
        ## p(G3|G1)
        G3_p = pm.Deterministic('G3_p', pm.math.switch(G1, 0.9, 0.1))
        G3 = pm.Bernoulli('G3', G3_p)

        X2 = pm.Normal('X2', mu=pm.math.switch(G2, 60, 50), sd=3.16)

        X3 = pm.Normal('X3', mu=pm.math.switch(G3, 60, 50), sd=3.16)
        trace = pm.sample(400000, tune=50, progressbar=True)

    X2_50 = 0
    G1 = 0
    X3_50 = 0
    for sample in trace:
        if 49.5 <= sample['X2'] <= 50.5:
            X2_50 += 1
            if sample['G1'] == 1:
                G1 += 1
            if 49.5 <= sample['X3'] <= 50.5:
                X3_50 += 1

    print "X2:", X2_50, " G1:", G1, " X3:", X3_50

    print "P(G1==2|X2==50): ", G1/X2_50
    print "P(X3==50|X2==50): ", X3_50/X2_50


if __name__ == '__main__':
    part2()
