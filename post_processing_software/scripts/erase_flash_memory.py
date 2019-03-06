#! /usr/bin/python3
import serial

def eraseFlashMemoryOnVirtualGimbal():
    ser = serial.Serial('/dev/ttyVIG0', timeout=1)
    print('Device:' + ser.name)
    print('Erasing angular velocity data on a flash memory...')
    ln = b''
    while 1:
        line = ser.readline()
        if len(line ) == 0 :
            break
        ln = ln + line

    ln = b''
    ser.write(b'u')
    ser.write(b'e')
    ser.write(b'y')
    ser.readline()
    while 1:
        line = ser.readline()
        if len(line ) == 0 :
            break
        ln = ln + line

    ser.close()
    return ln.decode()

if __name__ == '__main__':
    print('This script erases all data in flash memory of VirtualGimbal.')
    print('Do you continue? [y/N]')
    a = input()
    if a == 'y' or a == 'Y':
        print(eraseFlashMemoryOnVirtualGimbal())
        print('Done.')
    else:
        print('Canceled.')   
            
    