#!/usr/bin/python
from __future__ import print_function

import numpy as np
from twisted.internet.protocol import Factory
from twisted.internet import protocol
import pyqtgraph as pg
from Queue import Queue
import os, sys, time
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import qt4reactor


class RovertoClientProtocol(protocol.Protocol):

    def __init__(self):
        self.image_queue = Queue()

    def connectionMade(self):
        self.buf = b""
        self.factory.clientReady(self)
    
    def dataReceived(self, data):
        self.buf = self.buf + data
        if len(self.buf)>=(144*176):
            print("Image received")
            image = np.rot90(np.fromstring(self.buf[:144*176], dtype=np.uint8).reshape((176, 144)),2)
            self.image_queue.put(image)
            self.buf = self.buf[144*176:]
    
    def connectionLost(self, reason):
        print("Connection lost")

class RovertoClientFactory(protocol.ClientFactory):
    protocol = RovertoClientProtocol

    def __init__(self):
        self.client = None

    def clientConnectionFailed(self, connector, reason):
        print("Connection failed - goodbye!")
    
    def clientConnectionLost(self, connector, reason):
        print("Connection lost - goodbye!")

    def clientReady(self, client):
        print('Client ready')
        self.client = client

    def getImage(self):
        if not self.client:
            return None
        img = None
        try:
            img = self.client.image_queue.get(True, 0.1)
        except Exception, e:
            pass
        return img

class LogWidget(QTextBrowser):
    def __init__(self, parent=None):
        super(LogWidget, self).__init__(parent)
        palette = QPalette()
        palette.setColor(QPalette.Base, QColor('#000000'))
        self.setTextColor(QColor('#F2F2F2'))
        self.setPalette(palette)

class RovertoClientGUI(QMainWindow):
    def __init__(self, reactor, parent=None):
        super(RovertoClientGUI, self).__init__(parent)
        self.reactor = reactor

        self.host = "intermedio.ddns.net"
        self.port = 3500

        self.create_main_frame()
        
        self.client = RovertoClientFactory()
        self.create_timer()

    def create_main_frame(self):
        self.doit_button = QPushButton('Connect')
        self.doit_button.clicked.connect(self.connect)
        self.log_widget = LogWidget()
        
        win = pg.GraphicsLayoutWidget()
        win.setWindowTitle('Roverto camera')
        view = win.addViewBox()
        self.image_widget = pg.ImageItem(border='w')
        view.addItem(self.image_widget)
        #view.setAspectLocked(True)
       
        

        hbox = QHBoxLayout()
        
        hbox.addWidget(self.doit_button)
        hbox.addWidget(self.log_widget)
        hbox.addWidget(win)

        main_frame = QWidget()
        main_frame.setLayout(hbox)

        self.setCentralWidget(main_frame)

    def create_timer(self):
        self.circle_timer = QTimer(self)
        self.circle_timer.timeout.connect(self.update_image)
        self.circle_timer.start(1000)

    def update_image(self):
        img = self.client.getImage()
        if img is not None:
            self.log('Image received!')
            self.image_widget.setImage(img)
        

    def connect(self):
        self.log('Connecting...')
        self.connection = self.reactor.connectTCP(self.host, self.port, self.client)

    def on_client_connect_success(self):
        self.log('Connected to server.')

    def on_client_connect_fail(self, reason):
        # reason is a twisted.python.failure.Failure  object
        self.log('Connection failed: %s' % reason.getErrorMessage())

    def on_client_receive(self, msg):
        self.log('Client reply: %s' % msg)
        self.log('Disconnecting...')
        self.connection.disconnect()

    def log(self, msg):
        timestamp = '[%010.3f]' % time.clock()
        self.log_widget.append(timestamp + ' ' + str(msg))

    def closeEvent(self, e):
        self.reactor.stop()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    qt4reactor.install()
    from twisted.internet import reactor
    mainwindow = RovertoClientGUI(reactor)
    mainwindow.show()

    reactor.run()
