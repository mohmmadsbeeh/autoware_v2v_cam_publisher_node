#!/usr/bin/env python3
#------------------------------------------------------------------------------
# File       : fwd_wsmp_forward_tx_test.py
# Authors    : Paul Gray <paul.gray@cohdawireless.com>
# Company    : Cohda Wireless Pty. Ltd.
#   
#------------------------------------------------------------------------------
# Description: Simple routines to transfer UDP packets to the Cohda MK1 radio.
#              Used to test the fwd_wsmp_forward_tx utility, which listens
#              for packets on a given port, and encapsulates all packets
#              received on this port and transmits them over the air
#------------------------------------------------------------------------------
# Copyright (c) 2010 Cohda Wireless Pty Ltd
#-----------------------------------------------------------------------------

#####################
# modules to import
import time             # provides time functions
import socket           # provides socket functions
import sys              # system exception handling

#####################
# Global settings



#####################
# Local routines

#------------------------------------------------------------------------------
# Print the correct usage of this program
def PrintUsage(ProgramName):
    """
    Prints the correct usage of this script.
    """
    print ( 
       " Usage: %s <MK1Addr> <MK1Port> <PktRate> <PktLen> <NumPkts> \
        Where: \
       <MK1Addr> is the IP address of the MK1, e.g. 192.168.52.227 \
       <MK1Port> is the IP port of the MK1, e.g. 4040 \
       <PktRate> is the number of packets per second to transmit \
       <PktLen>  is the length of the payload \
       <NumPkts> is the number of packets to transmit (-1 = forever)" \
     % ProgramName )

######
def print_byte_string(bytes,
                      start_addr = 0,
                      bytes_per_line = 16):
    """
    Nicely print a byte string as formatted rows of hexadecimal bytes,
    with addresses.
    """

    addr = start_addr
    print ("0x%04x:" % addr )
    for i in range(len(bytes)):
        print ( '%02x ' % ord(bytes[i]) )
        addr = addr + 1
        if (i % bytes_per_line) == (bytes_per_line-1):
            print ()
            if i != (len(bytes)-1):
                print ( "0x%04x:" % addr)
 
    print ()



#####################
# Main routines

#-------------------------------------------
# Packet Source
def TestPacketSource(mk1_addr,            # IP Address of MK1 Radio
                     mk1_port = 4040,     # Port of MK1 Radio
                     pkt_rate = 1,        # Number of packets/sec to gen
                     pkt_len = 100,       # Packet length
                     num_pkts = -1):      # Num packets to generate (-1=forever)
    """
    Test packet source routine, which generates UDP packets to be forwarded by
    the fwd_wsmp_forward_tx utility.

    This requires that the utility 'fwd_wsmp_forward_tx' is running on the 
    target radio.

    Arguments:
    mk1_addr - the IP Address of the MK1 Radio used to transmit packet
               e.g. '192.168.227.227', or '255.255.255.255'
    mk1_port - the port of the MK1 Radio used to transmit packet
               e.g. 4040
    pkt_rate - the number of packets per second to generate
    pkt_len  - Number of bytes in the packet
    num_pkts - Number of packets to generate (-1 means forever)
    """

    PktPeriod = 1.0/pkt_rate     # Period of packet generation
    sleeptime = PktPeriod        # time to sleep between packets

    print ("Test source to MK1 on %s:%d" % (mk1_addr, mk1_port))

    # Open the socket to communicate with the mk1
    txsock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # create socket
    txsock.bind(('', 50000)) # bind to whatever port we can get our hands on
    txsock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    Target = (mk1_addr, mk1_port) # the target address and port

    new_time = time.time()
    last_time = new_time
    end_time = time.time()


    # initialiase some variables to time packets
    start_pkt_num = 0
    last_pkt_num = 0
    pkt_count = 0

    start_time = new_time
    last_update_time = time.time()
    start_time = last_update_time

    UpdatePeriod = 1.0
    PktRatePeriod = 1.0

    try:
        # Main loop
        while (pkt_count < num_pkts) or (num_pkts==-1):
            # record the time the packet is generated
            pkt_time = time.time()

            # Generate the UDP header
            src_port = 0    # we don't expect replies
            check_sum = 0   # don't worry about the checksum
            udp_length = pkt_len + 8
            pktbuf = bytearray()
            #pktbuf = pktbuf.append((src_port >> 0) & 0xff)
            #pktbuf = pktbuf.append((src_port >> 8) & 0xff)
            #pktbuf = pktbuf.append((mk1_port >> 0) & 0xff)
            #pktbuf = pktbuf.append((mk1_port >> 8) & 0xff)
            #pktbuf = pktbuf.append((udp_length >> 0) & 0xff)
            #pktbuf = pktbuf.append((udp_length >> 8) & 0xff)
            #pktbuf = pktbuf.append((check_sum >> 0) & 0xff)
            #pktbuf = pktbuf.append((check_sum >> 8) & 0xff)
            # Generate the payload (just an incrementing sequence)
            for i in range(0, pkt_len):
                pktbuf.append(i & 0xff)

            #print ('- Transmitting packet with payload: -')
            #print_byte_string(pktbuf)
            print ('Total packets transmitted: %d' % (pkt_count+1))
            #print ('')

            # Transmit the packet to the MK1 via the socket
            txsock.sendto(pktbuf, Target)

            # Increment packet numbers
            pkt_count = pkt_count + 1

            if (new_time - last_update_time > UpdatePeriod):
                tx_pkt_rate = ((pkt_count - last_pkt_num) / 
                               (new_time - last_update_time))
                last_pkt_num = pkt_count
                last_update_time = new_time

            # Keep adjusting sleep time to keep our rate to desired level,
            # averaged over PktRatePeriod intervals
            last_time = new_time
            new_time = time.time()

            timediff = new_time - last_time

            errtime = (PktPeriod*(pkt_count - start_pkt_num) - 
                       (new_time - start_time))
            sleeptime = PktPeriod + errtime

            if (new_time - start_time > PktRatePeriod):
                start_time = new_time
                start_pkt_num = pkt_count

            if sleeptime < 0:
                sleeptime = 0
            time.sleep(sleeptime)
        txsock.close()
        print ("Transmitted %d packets" % (pkt_count))
        return (pkt_count)
            
    except KeyboardInterrupt:
        txsock.close()
        print ('User CTRL-C, stopping packet source' )
        sys.exit(1)

    except:
        txsock.close()
        print ("Got exception:", sys.exc_info()[0])
        raise
           
#------------------------------------------------------------------------------
# Main Program
#------------------------------------------------------------------------------
if __name__ == '__main__':

    if len(sys.argv) != 6:
        PrintUsage(sys.argv[0])
        sys.exit(1)
    
        
    mk1_addr = sys.argv[1]
    mk1_port = int(sys.argv[2])
    pkt_rate = int(sys.argv[3])
    pkt_len = int(sys.argv[4])
    num_pkts = int(sys.argv[5])

    TestPacketSource(mk1_addr, mk1_port, pkt_rate, pkt_len, num_pkts)