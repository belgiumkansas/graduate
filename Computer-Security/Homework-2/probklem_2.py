
import collections
from itertools import cycle

ALPHA = 'abcdefghijklmnopqrstuvwxyz'

def encrypt(plaintext, key):

    key_pairs = zip(plaintext, cycle(key))
    result = ''

    for pair in key_pairs:
        shift = reduce(lambda x, y: ALPHA.index(x) + ALPHA.index(y), pair)
        result += ALPHA[shift % 26]
    return result


def pop_var(X, mu):
    N = len(X)

    var_sum = 0
    for i in range(0,N):
        var_sum += (X[i]-mu )*(X[i]-mu )

    return var_sum/N



def main():

    frequency = {"A": 8.167, "B": 1.492, "C": 2.782, "D": 4.253, "E": 12.702,
        "F": 2.228, "G": 2.015, "H": 6.094, "I": 6.996, "J": 0.153, "K": 0.772,
        "L": 4.025, "M": 2.406, "N": 6.749, "O": 7.507, "P": 1.929, "Q": 0.095,
        "R": 5.987, "S": 6.327, "T": 9.056, "U": 2.758, "V": 0.978, "W": 2.360,
        "X": 0.150, "Y": 1.974, "Z": 0.074}

    file = open("plaintext","r")
    plaintext = file.read()
    plaintext = plaintext.replace('\n','')


    X = frequency.values()
    mu = float(100)/26

    print "part a:", pop_var(X ,mu)

    N = len(plaintext)
    letters = collections.Counter(plaintext)

    for key, value in letters.items():
        letters[key] = float(value*100)/N

    X = letters.values()

    print "part b:", pop_var(X, mu)

    keys = ["yz", "xyz", "wxyz", "vwxyz", "uvwxyz"]

    for i in range(0, len(keys)):
        #encrypt the text with key
        encrypted_text = encrypt(plaintext, keys[i])
        print "key used", keys[i]

        #setup for splitting text into bins for each key
        N = len(encrypted_text)
        key_length = len(keys[i])
        print key_length
        string_bins = ["" for x in range(key_length)]
        bin_count = 0
        #split text into bins based on which key was
        #used to encrypt it
        for j in encrypted_text:
            string_bins[bin_count] += j
            bin_count = (bin_count + 1)%key_length
        var_average = 0
        #do frequency calculations for each bin
        for j in range(0,key_length):
            N = len(string_bins[j])
            letters = collections.Counter(string_bins[j])
            for key,value in letters.items():
                letters[key] = float(value*100)/N
            X = letters.values()
            var = pop_var(X, mu)
            print "var of bin", j,"=", var
            var_average += var/key_length
        #average them out
        print "average var:", var_average

        '''
        letters = collections.Counter(encrypted_text)
        for key, value in letters.items():
            letters[key] = float(value*100)/N
        X = letters.values()
        print "key:", keys[i], "varience:", pop_var(X, mu)
        '''






if __name__ == '__main__':
    main()
