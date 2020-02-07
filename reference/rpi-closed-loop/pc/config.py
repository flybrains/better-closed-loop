# ==============================================================================
# IP and port for the laptop, to establish socket communication with
# FicTrac. Get the IP by running ipconfig on this computer, and look
# for the port in the FicTrac configuration file.
LOCAL_HOST = '192.168.137.1'
LOCAL_PORT = 3100
# ==============================================================================
# IP and port for connecting to Raspberry Pi. Set the port manually here
# Get the IP by running ifconfig on the Raspberry Pi.
RPI_HOST = '192.168.137.10'
RPI_PORT = 5000
# ==============================================================================
# Replay Parameters
REPLAY = False
REPLAY_FOLDER = r'C:\Users\rutalab\Desktop\logs-to-replay'
# ==============================================================================
# If you want to run a closed loop experiment with constant flow, set True
# and provide a flowrate (Max = 666 mL/min)
CONSTANT_FLOW = True
FLOW_RATE = 400
PERCENT_AIR = 100
PERCENT_ACV = 0
# ==============================================================================
# If you want to run a closed loop experiment with propoartional odor, set True
# and provide the number of sources and their x/y positions.
# If number of sources is 1, the parameters of source 2 are disregarded,
# as well as the x position for source 1.
PROPORTIONAL_ODOR = False
MAX_TOTAL_AIRFLOW = 400
N_ODOR_SOURCES = 1
SINGLE_SOURCE_DISTANCE= 500 #SET ONLY THIS ONE IF DOING SINGLE SOURCE EXPERIMENT
REPEATING_STRIPS = False
XON = 100 # Strip width
REINFORCEMENT_AT_END = 3.0
# ==============================================================================
# If you want to activate LEDs at any time, set LED activation to True
# Set LED intensity as a fraction of maximum voltage
LED_ACTIVATION = False
LED_COLOR = 'red'
LED_INTENSITY = 1.0 #NOTE THIS NUMBER MUST BE LESS THAN OR EQUAL TO 1 (1 = MAX INTENSITY, 0.5 = HALF)
# ............................................................................
# LED stimulation can be temporally-activated (on a fixed program) or
# conditionally-activated (based on a behavioral characteristic)
STIMULATION_MODE = 't'    # t for temporal, c for conditional
# ............................................................................
# If you choose to use LEDs and use them temporally, fill out out the following.
INITIAL_LED_DELAY = 1
PERIOD_BETWEEN_PULSE_RISING_EDGES = 3
DURATION_OF_PULSE_T = 1
# ............................................................................
# If you choose to use LEDs and use them conditionally, fill out out the following.
# The following behavioral characteristics can be used to activate light
#   BINARY CONDITIONS
#       1   UPWIND_TRACKING     [Y/N]
# Pick a condition to use and set the number
CONDITION_TO_USE = 1

# Set a threshold value to consider the condition met. This is different based
# on the kinematic characteristic
CONDITION_THRESHOLD = 3
SLIDING_WINDOW_LENGTH = 10

# Set the following light parameters
DURATION_OF_PULSE_C = 1
LOCKOUT_TIME_AFTER_PULSE = 15
# ==============================================================================
#
#
#   DON'T ADJUST PARAMETERS BELOW THIS LINE
#
#
# ==============================================================================
import sys
import numpy as np

def check_cfg():
    if CONSTANT_FLOW and PROPORTIONAL_ODOR:
        print('Both CONSTANT_FLOW and PROPORTIONAL_ODOR are set to True')
        print('Please pick one or the other')
        sys.exit()

    if N_ODOR_SOURCES > 1:
        dist = np.linalg.norm(np.asarray([SOURCE_1_Y_DISTANCE, SOURCE_1_X_DISTANCE]) - np.asarray([SOURCE_2_Y_DISTANCE, SOURCE_2_X_DISTANCE]))
        if dist <= np.abs(SOURCE_1_RADIUS - SOURCE_2_RADIUS):
            print('The range of one odor source is contained entirely by the other')
            print('This is not a supported experiment configuration')
            print('Please adjust the positioning of odor sources')
            sys.exit()

    return None

















#
