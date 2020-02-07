
import serial
import time


def initialize_arduino():
    arduino = serial.Serial("/dev/ttyACM0", 38400)
    arduino.baudrate = 38400
    time.sleep(1)
#    arduino.write("<a>".encode())
 #   time.sleep(1)
    return arduino

def send_to_arduino(arduino, value):
    sendstr = "<"+str(value)+">\n"
    arduino.write(sendstr.encode())
    print('Sent {}'.format(sendstr))
    return None

def kill_arduino(arduino):
  #  arduino.write("<a>".encode())
   # time.sleep(1)
    arduino.close()
    return None
