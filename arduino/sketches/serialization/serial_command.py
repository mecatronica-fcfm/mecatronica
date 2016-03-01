#!/usr/bin/env python

import serial
import sys

def serial_read(port = '/dev/ttyUSB0', baudrate = 9600):
  ser = serial.Serial(port, baudrate, timeout = 1)

  header = '{'
  tail = '}'
  idx = 0

  print 'Waiting for header: ' + str(header)
  while True:
    c = ser.read(1)
    if c == header:
      try:
        # Leer una linea (ie. hasta \r\n)
        line = ser.readline()
        # Borrar caracteres de fin de linea
        line = line.strip()
        # Tail check
        if not line[-1] == tail:
          print 'Unexpected tail'
          continue
        # Obtener campos
        raw_data = line[:-1].split(',')
        print raw_data
        
      # Salir con Ctrl+C
      except KeyboardInterrupt:
        print 'Exit'
        ser.close()
        sys.exit()
      except:
        print 'Error'
      


if __name__ == '__main__':
  serial_read()

