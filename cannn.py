import serial
import serial.tools.list_ports as lst
import time

info = lst.comports()

ser = serial.Serial()

message_counter = 0
sent_counter = 0
lost_messages = 0

def find_Stm():
    for port in info:
        if(port.product.find("STM32") != -1):
            return port.device
    return 0

def open_device(dev):
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 2250000
    ser.open()

def parse_data(data):
    global sent_counter, message_counter, lost_messages
    data_split = data.split(' ')
    sent_counter = int(data_split[9]) + int(data_split[8]) * 256 + int(data_split[7]) * 256 * 256
    if (message_counter == 0):
        message_counter = sent_counter
    message_counter += 1
    lost_messages = sent_counter - message_counter

# print("main")
# if find_Stm() != 0:
#open_device(find_Stm())
open_device('as')
print("Found STM")
while True:
    data = ser.readline()
    data = data.decode('utf-8')
    data = data.replace('\t', ' ')
    data = data.replace('\r\n', '')
    print(data)
    # parse_data(data)
    # print("{} ---------- Lost Messages: {}".format(data, lost_messages))
# else:
#         print("no STM32 Detected, Exit_Program")
#         exit(0)
