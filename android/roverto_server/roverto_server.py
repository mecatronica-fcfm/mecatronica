#!/usr/bin/python
from __future__ import print_function
from twisted.internet import reactor, protocol, endpoints
import argparse

class RovertoServerProtocol(protocol.Protocol):
    def __init__(self, factory):
        self.factory = factory
        self.peer = None

    def connectionMade(self):
        self.peer = self.transport.getPeer()
        print("New connection from: {}:{}".format(self.peer.host,self.peer.port))
        self.factory.clients.add(self)

    def connectionLost(self, reason):
        print("Connection closed from: {}:{}".format(self.peer.host,self.peer.port))
        self.factory.clients.remove(self)

    def dataReceived(self, data):
        for c in self.factory.clients:
            if c != self:
                c.transport.write(data)

class RovertoServerFactory(protocol.Factory):
    def __init__(self):
        self.clients = set()

    def buildProtocol(self, addr):
        return RovertoServerProtocol(self)

class RovertoServer(object):
    def __init__(self, port=3500):
        self.port = port
        endpoints.serverFromString(reactor, "tcp:{}".format(self.port)).listen(RovertoServerFactory())
    
    def run(self):
        print("Running RovertoServer at {} port".format(self.port))
        reactor.run()


def main():
    parser = argparse.ArgumentParser(prog='roverto_server', description='Roverto TCP server')
    parser.add_argument('-p','--port', type=int, default=3500, help='TCP port (default: 3500)')
    args = parser.parse_args()
    
    server = RovertoServer(args.port)
    server.run()

if __name__ == '__main__':
    main()
