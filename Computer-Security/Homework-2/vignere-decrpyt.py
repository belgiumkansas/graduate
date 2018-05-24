from math import log

import re



#return factors of a number
def Factorize(n):
      return set(reduce(list.__add__, ([i, n//i] for i in range(1, int(n**0.5) + 1) if n % i == 0)))

#returns dictionary of repeated substrings and their position
def RepeatedSubstrings(cipher_text, sub_length):
    subs = {}
    for i in range(0,len(cipher_text) - sub_length):
        sub_string = cipher_text[i:i+sub_length]
        if cipher_text.count(sub_string) > 1 and not sub_string in subs.keys():
            subs[sub_string] = [m.start() for m in re.finditer(sub_string, cipher_text)]
    return subs

#uses repeated substrings to find most likely key length
def Possible_Keylengths(cyphertext):
    #find the repeated substrings and location use 3 because kasiski
    substrings_repeat = RepeatedSubstrings(cyphertext, 3)
    #sotre the distances between repeated substrings
    lengths = []
    for substring in substrings_repeat.keys():
        for i in range(0, len(substrings_repeat[substring])-1):
            lengths.append(substrings_repeat[substring][i+1] - substrings_repeat[substring][i])
    #dictionary of how often a factor of a repeat emerges
    count_dict = {}
    #make a dictionary count of factors for all lengths
    for d in lengths:
        factors = Factorize(d)
        for f in factors:
            if not f in count_dict.keys(): count_dict[f] = 1
            else: count_dict[f] += 1
    #sort from most ot least frequenct and return
    return sorted(count_dict, key= count_dict.__getitem__, reverse=True)

#return english match %
def english_match(text):

    #print "Cracking...\n{0}".format(text)
    text = text.lower()


    with open(pathto_dictionary) as f:
        dictionary = [x.strip('\n') for x in f.readlines()]


def infer_spaces(text, words, wordcost, maxword):


    #find the best match for ith first characters
    def best_match(i):
        candidates = enumerate(reversed(cost[max(0, i-maxword):i]))
        return min((c + wordcost.get(text[i-k-1:i], 9e999), k+1)for k,c in candidates)

    #cost array
    cost = [0]
    for i in range(1,len(text)+1):
        c,k = best_match(i)
        cost.append(c)

    #backtrack for minimal cost string
    out = []
    i = len(text)
    while i > 0:
        c,k = best_match(i)
        assert c == cost[i]
        out.append(text[i-k:i])
        i -= k
    #print cost[len(cost)-1]
    return (" ".join(reversed(out)), cost[len(cost)-1])

def Decrypt(text,key):

	char_map = map(chr, range(97 , 123))
	m = len(key)
	decrypt = ""

	for i in range(len(text)-1):
		ascii_val = char_map.index(text[i])
		key_shift =  char_map.index(key[i % m])
		value = (ascii_val - key_shift) % len(char_map)
		decrypt += char_map[value]

	return decrypt


def main():
    #dict_path = sys.args[1]
    dictionary = "words-by-frequency.txt"

    #open, read and format cypher text
    file = open("ciphertext","r")
    cyphertext = file.read()
    cyphertext = cyphertext.replace('\n','')
    cyphertext = cyphertext.lower()
    #open read and format plain text
    file = open("plaintext","r")
    plaintext = file.read()
    plaintext = plaintext.replace('\n','')


    lowest_score = 5000

    words = open(dictionary).read().split()
    wordcost = dict((k, log(i+1*log(len(words)))) for i,k in enumerate(words))
    maxword = max(len(x) for x in words)


    for key in words:

        if len(key) == 7:
            test_decrypt = Decrypt(cyphertext, key)
            english_check = infer_spaces(test_decrypt, words, wordcost, maxword)

            if english_check[1] <= lowest_score:
                lowest_score = english_check[1]
                #print lowest_score
                print english_check[0]
                print "current best key:", key, "score:", lowest_score


    print "finsihed"













    #print plaintext
    #returns most likely keylengths ordered
    #keylengths = Possible_Keylengths(plaintext)

    #solution = infer_spaces(plaintext.lower(), dict_path)
    #print solution[1]

    #english_match(plaintext, dict_path )

if __name__ == '__main__':
    main()
