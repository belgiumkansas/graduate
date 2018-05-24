import dpkt
import sys
import struct 
import socket

#ip_list[ipaddress][num_SYN, num_SYNACK]

def getip(s):
    i = struct.unpack(">L", bytearray(s))[0]
    return socket.inet_ntoa(struct.pack(">L", i)) 

def main():
    filename = sys.argv[-1]
    f = open(filename)
    pcap = dpkt.pcap.Reader(f);
    ip_list = {}
    j = 1

    for ts, buf in pcap:
        # show we are making progress
        if j % 1000000 == 0 :
            print "Parsed " + str(j) + " data packets..." 
        j += 1
        try:
            eth = dpkt.ethernet.Ethernet(buf)        
            ip = eth.data
            tcp = ip.data

            if ip.p == dpkt.ip.IP_PROTO_TCP:
                if tcp.flags & dpkt.tcp.TH_SYN and not tcp.flags & dpkt.tcp.TH_ACK:
                    if not ip.src in ip_list:
                        ip_list[ip.src] = [1, 0]
                    else:                
                        ip_list[ip.src][0] += 1
                                 
                elif tcp.flags & dpkt.tcp.TH_SYN and tcp.flags & dpkt.tcp.TH_ACK:
                    if not ip.dst in ip_list:
                        ip_list[ip.dst] = [0, 1]
                    else:
                        ip_list[ip.dst][1] += 1

        except:
            continue

    for k, v in ip_list.iteritems():
        if v[0] > 3 * v[1]:
            print getip(k)

if __name__ == "__main__":
    main()
