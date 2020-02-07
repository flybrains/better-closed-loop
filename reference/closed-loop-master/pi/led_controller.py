from gpiozero import PWMLED
from time import sleep

def initialize_led():

    led_pin1 = 23 #red
    led_pin2 = 24 #green

    l1 = PWMLED(led_pin1)
    l2 = PWMLED(led_pin2)

    l1.value = 0.0
    l2.value = 0.0

    return l1, l2

def change_led_dutycycles(l1, l2, ds1, ds2):
    l1.value = ds1
    l2.value = ds2

    return l1,l2

if __name__=="__main__":
    l1, l2 = initialize_led()
    l1.value = 0.5
    l2.value = 0.1
    sleep(3)
    l2.value = 1.0
    sleep(5)
    
