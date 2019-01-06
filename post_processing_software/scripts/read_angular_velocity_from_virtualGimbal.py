#! /usr/bin/python3
import serial
ser = serial.Serial('/dev/ttyVIG0', timeout=1)
print(ser.name)
ln = b''
while 1:
    line = ser.readline()
    if len(line ) == 0 :
        break
    ln = ln + line
# print(ln.splitlines())
print(ln.decode()) # Convert bytes to string

ln = b''
ser.write(b'j')
while 1:
    line = ser.readline()
    if len(line ) == 0 :
        break
    ln = ln + line
# print(ln.splitlines())
print(ln.decode()) # Convert bytes to string


ser.close()
