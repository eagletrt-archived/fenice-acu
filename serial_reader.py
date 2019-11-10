import serial
import serial.tools.list_ports as lst
import time

ser = serial.Serial()
print("avaiable ports are:")
ports=list(lst.comports())
for p in ports:
	print(p)

ser.port = '/dev/ttyACM0'
ser.baudrate = 2250000
ser.open()

while True:
    data = ser.readline()