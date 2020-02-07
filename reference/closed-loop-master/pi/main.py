import sys
import time
from numpy import abs
from socket import socket, AF_INET, SOCK_DGRAM, gethostbyname

from gpiozero import PWMLED
from led_controller import initialize_led
from socket_utils import initialize_socket_connection, poll_socket, split_data, check_data_for_cfg
from arduino_utils import initialize_arduino, send_to_arduino, kill_arduino

def main():
    arduino = initialize_arduino()
    socket = initialize_socket_connection(5000)
    mfc1 = PWMLED(4)
    mfc2 = PWMLED(27)
    mfc3 = PWMLED(22)
    mfc1.value = 0.0
    mfc2.value = 0.0
    mfc3.value = 0.0

    led1, led2 = initialize_led()
    lastLED1sp, lastLED2sp = 0.0, 0.0
    lastMFC1sp, lastMFC2sp, lastMFC3sp = 0.0, 0.0, 0.0
    last_motor_target = 800000
    runstate = 1
    try:
        while True:
            item = poll_socket(socket)
            print(item)
            runstate, setpoints = split_data(item)
            if runstate==0:
                kill_arduino(arduino)
                led1.value = 0.0
                led2.value = 0.0
                mfc1.value = 0.0
                mfc2.value = 0.0
                mfc3.value = 0.0
                break
            else:
                [motor_target, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp] = setpoints
                if (led1_sp!=lastLED1sp) or (led2_sp!=lastLED2sp):
                    #led1.value = led1_sp
                    #led2.value = led2_sp
                    lastLED1sp = led1_sp
                    lastLED2sp = led2_sp

                if (mfc1_sp!=lastMFC1sp) or (mfc2_sp!=lastMFC2sp) or (mfc3_sp!=lastMFC3sp):
                    mfc1.value = mfc1_sp
                    mfc2.value = mfc2_sp
                    mfc3.value = mfc3_sp
                    lastMFC1sp = mfc1_sp
                    lastMFC2sp = mfc2_sp
                    lastMFC3sp = mfc3_sp
                if abs(motor_target - last_motor_target) >= 4:
                    print("-")
                    send_to_arduino(arduino, motor_target)
                    last_motor_target = motor_target
    except KeyboardInterrupt:
        kill_arduino(arduino)
        led1.value = 0.0
        led2.value = 0.0
        mfc1.value = 0.0
        mfc2.value = 0.0
        mfc3.value = 0.0
if __name__=='__main__':
    main()
