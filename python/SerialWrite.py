
import serial
import sys

def sendBinaryByte():
    byte = int(raw_input('Byte Binary Value = '), 2)
    print 'Bytes written: ', ser.write(chr(byte))
    
def sendDecByte():
    byte = int(raw_input('Byte Decimal Value = '))
    print 'Bytes written: ', ser.write(chr(byte))
    return
def sendString(sufix):
    byteArray = raw_input('String Value = ')+sufix
    print 'Bytes written: ', ser.write(bytes(byteArray))
    return

# Serial Port Configuration
try:
    ser = serial.Serial('COM4')

    print '-------------------------'
    print 'Serial Port Configuration'
    print 'Port is: '+ser.name
    print 'Open Port:', ser.isOpen()
    print 'Baudrate is: '+str(ser.baudrate)
    print 'Byte size is: '+str(ser.bytesize)
    print 'Parity Bit:',ser.parity
    print 'Stop Bits:',ser.stopbits
    print 'Read Timeout:', ser.timeout, 'seconds'
    print 'Write Timeout:', ser.writeTimeout, 'seconds'
    print '-------------------------\n'

except:
    print 'Inexpected error opening serial port', sys.exc_info()[0]
    ser.close()

# Menu Selection
while True:
    
    print '\nSelect Action:'
    print '0) Exit'
    print '1) Send Byte (binary)'
    print '2) Send Byte (dec)'
    print '3) Send String (no newline)'
    print '4) Send String (w/newline)'

    input_string = raw_input('Menu Selection: ')

    if(input_string == '1'):
        sendBinaryByte()
    elif(input_string == '2'):
        sendDecByte()
    elif(input_string == '3'):
        sendString('')
    elif(input_string == '4'):
        sendString('\n')
    elif(input_string == '0'):
        break
    else:
        pass
    
ser.close()

