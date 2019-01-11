#! /usr/bin/python3
import serial

def readAngularVelocityFromVirtualGimbal():
    ser = serial.Serial('/dev/ttyVIG0', timeout=1)
    print(ser.name)
    ln = b''
    while 1:
        line = ser.readline()
        if len(line ) == 0 :
            break
        ln = ln + line

    ln = b''
    ser.write(b'j')
    ser.readline()
    while 1:
        line = ser.readline()
        if len(line ) == 0 :
            break
        ln = ln + line
    # print(ln.decode()) # Convert bytes to string

    ser.close()

    return ln.decode()

if __name__ == '__main__':
    print(readAngularVelocityFromVirtualGimbal())