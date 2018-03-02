#!/usr/bin/env python

import socket
import struct

# see https://stackoverflow.com/questions/603852/multicast-in-python
class UdpMulticastSender:
  def __init__(self, MCAST_GRP = '224.1.1.1', MCAST_PORT = 5007):
    self.MCAST_GRP = MCAST_GRP
    self.MCAST_PORT = MCAST_PORT
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    
  def send(self, msg):
    self.sock.sendto(msg, (self.MCAST_GRP, self.MCAST_PORT))


class UdpMulticastReceiver:
  def __init__(self, MCAST_GRP = '224.1.1.1', MCAST_PORT = 5007):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
                                 # to MCAST_GRP, not all groups on MCAST_PORT
    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

  def recv(self, bufsize=4096):
    return self.sock.recv(bufsize)
