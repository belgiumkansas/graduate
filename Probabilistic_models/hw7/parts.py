#!/usr/bin/env python2
from __future__ import division

from sklearn.decomposition import LatentDirichletAllocation
from sklearn.feature_extraction.text import TfidfVectorizer, CountVectorizer

import math
import random
import numpy as np
import numpy.random
import sys


def generate_data(M=200, D=20, K=3, alpha=.1, beta=.01, L=50):
    ## Inputs:
    # M = number of documents
    # D = number of words in the corpus
    # K = number of topics
    # alpha, beta = arguments for the dirichlet distribution
    # L = number of words in each document

    ## Outputs:
    # data = a matrix with the following columns:
    #   - name of the document
    #   - name of the word
    #   - frequency of the word in the document

    # mixing = an MxK matrix. The ith row is the distribution of topics for the ith element
    # topics = a KxD matrix. the kth row is the distribution of words in the kth topic.

    ## Generate the mixing measure:
    alpha = [0.01 for i in range(K)]
    mixing = np.random.mtrand.dirichlet(alpha, size=M) # theta values

    ## Generate the topics:
    beta = [0.1 for i in range(D)]
    topics = np.random.mtrand.dirichlet(beta, size=K) # phi values

    ## For each of the word positions i, j where i in {1, ..., M} and j in {1, ..., N}
    # Choose topic = Multinomial(theta)
    Z = np.zeros(shape=(M, L))
    for i in range(M):
        for j in range(L):
            Z[i, j] = np.nonzero(np.random.multinomial(1, mixing[i,:]))[0][0]

    # Choose word = Multinomial(phi)
    data = np.zeros(shape=(M, D))
    for i in range(M):
        for j in range(L):
            phi = topics[int(Z[i, j]),:]
            idx = np.nonzero(np.random.multinomial(1, phi))[0][0]
            data[i, idx] += 1
    return data, mixing, topics, Z

def generate_document(data):
    alphabet = 'abcdefghijklmnopqrst'
    doc = ""
    for i in range(len(data)):
        if data[i] > 0:
            for j in range(int(data[i])):
                doc = doc + alphabet[i]
    # Shuffle the string
    doc_list = [a for a in doc]
    random.shuffle(doc_list)
    shuffled_doc = " ".join(doc_list)
    return shuffled_doc

def generate_numerical_document(data):
    alphabet = 'abcdefghijklmnopqrst'
    doc = ""
    for i in range(len(data)):
        if data[i] > 0:
            for j in range(int(data[i])):
                doc = doc +' '+ str(i)
    return doc[1:]


def part1():
    pass


def main():
    data, mixing, topics, z = generate_data()
    docs = []
    for i in range(200):
        docs.append(generate_numerical_document(data[i]))

    tf_vectorizer = CountVectorizer(max_features=20, stop_words=None)

    tf = tf_vectorizer.fit(docs)

    lda = LatentDirichletAllocation(n_components=3,
                                    total_samples=200,
                                    doc_topic_prior = 0.1,
                                    topic_word_prior=0.01,
                                    max_iter=5,
                                    learning_method='batch',
                                    random_state=0)
    lda.fit(data)
    print data.shape
    print data
    print lda.components_



if __name__ == '__main__':
    main()









    pass
