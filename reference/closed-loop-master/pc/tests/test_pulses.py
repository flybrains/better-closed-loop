import sys
import argparse
import time
import csv
import logging
import serial
import numpy as np
import config
from datetime import datetime
from socket import socket, AF_INET, SOCK_STREAM, SOCK_DGRAM

def establish_pi_connection(RPI_HOST, RPI_PORT):
	PACKET_SIZE = 1024
	sock=socket(AF_INET, SOCK_DGRAM)
	time.sleep(1)
	return sock

def convert_angle_for_arduino(inputVal, previousAngle, mult):

	inputVal = inputVal*(256/800)
	spr = 800
	conv1 = spr*(1000/256)
	newAngle1 = (inputVal*conv1)/1000

	highLimit = 500
	lowLimit = 300
	midPoint = 400
	offset = mult*spr

	if (newAngle1 < highLimit) and (newAngle1 > lowLimit):

		if(newAngle1 > midPoint):
			newAngle1 = highLimit
		else:
			newAngle1 = lowLimit


	newAngle1 = newAngle1 + offset

	if np.abs(newAngle1-previousAngle) > 400:
		if(newAngle1 > previousAngle):
			mult = mult-1
			offset = mult*spr
			newAngle1 = newAngle1 - spr

		else:
			mult = mult + 1
			offset = mult*spr
			newAngle1 = newAngle1 + spr

	previousAngle = newAngle1


	return int(newAngle1), mult

def initialize_logger():
	# create logger
	now = datetime.now() # current date and time
	dts = now.strftime("%m%d%Y-%H%M%S")
	logger = logging.getLogger('closed_loop_client_output')
	hdlr = logging.FileHandler(r'C:\Users\rutalab\Desktop\fictrac-data\client-logs\{}.log'.format(dts))
	logger.addHandler(hdlr)
	logger.setLevel(logging.DEBUG)
	logger.info('{} ` {}, {}, {}, {}, {}, {}, {}, {}, {}'.format("timestamp", "ft_heading", "ft_xPos","ft_yPos", "motor_step_command", "mfc1_stpt", "mfc2_stpt", "mfc3_stpt", "led1_stpt", "led2_stpt"))
	return logger

def parse_arguments():
	parser = argparse.ArgumentParser()
	parser.add_argument("odor_on_time")
	parser.add_argument("odor_off_time")
	parser.add_argument("n_cycles")
	args = parser.parse_args()
	return int(args.odor_on_time), int(args.odor_off_time), int(args.n_cycles)


def get_kinematic_values(data, previousAngle, mult):
	line = data.decode('UTF-8')
	toks = line.split(',')


	if ((len(toks) < 24) | (toks[0] != "FT")):
		sys.exit()

	posx = float(toks[15])*3
	posy = -float(toks[16])*3
	net_vel = float(toks[19])*3
	heading = float(toks[17])

	heading = -heading
	motorHeadingMap = heading % (2*np.pi)
	propHead = motorHeadingMap/(2*np.pi)
	target = int(propHead*800)

	correctedTarget, mult = convert_angle_for_arduino(target, previousAngle, mult)

	previousAngle, motorSendVal = correctedTarget, correctedTarget
	motorLastTime = time.time()

	return motorSendVal, motorLastTime, heading, posx, posy, mult, previousAngle

def close_and_clean(RPI_SOCK,RPI_HOST, RPI_PORT):
	SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(0, 'a', 0.0, 0.0, 0.0, 0.0, 0.0) +'>\n'
	RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), (RPI_HOST, RPI_PORT))
	RPI_SOCK.close()
	sys.exit()

def run_fictrac_client(LOCAL_HOST, LOCAL_PORT, RPI_HOST, RPI_PORT, odor_on_time, odor_off_time, n_cycles):

	RPI_SOCK  = establish_pi_connection(RPI_HOST, RPI_HOST)
	logger = initialize_logger()

	with socket(AF_INET, SOCK_STREAM) as sock:
		time.sleep(0.02)
		sock.connect((LOCAL_HOST, LOCAL_PORT))

		mfc1_sp, mfc2_sp, mfc3_sp = 0.0, 0.0, 0.0
		led1_sp, led2_sp = 0.0, 0.0
		motorSendVal, lastMotorSendVal = 800000, 800000
		expStartTime = time.time()
		LEDLastTime, odorLastTime, motorLastTime = expStartTime,expStartTime,expStartTime
		previousAngle = 800000
		mult = 1000

		odorOn=False
		odorOff=False
		odorActivationCount = 0

		try:
			i = 0
			while True:
				data = sock.recv(1024)
				if not data:
					break

				if (time.time() - motorLastTime) > (1/60000):

					led1_sp, led2_sp = 0.0, 0.0
					mfc1_sp = 0.0#empty
					motorSendVal, motorLastTime, heading, posx, posy, mult, previousAngle = get_kinematic_values(data, previousAngle, mult)

					if odorOn==False and odorOff==False:
						odorOn=True
						lastOdorStartTime = time.time()
					elif odorOn==True and (time.time()-lastOdorStartTime)<odor_on_time:
						pass
					elif odorOn==True and (time.time()-lastOdorStartTime)>=odor_on_time:
						odorOn=False
						odorOff=True
						lastOdorStopTime = time.time()
						odorActivationCount+=1
					elif odorOff==True and (time.time()-lastOdorStopTime)<odor_off_time:
						pass
					elif odorOff==True and (time.time()-lastOdorStopTime)>=odor_off_time:
						odorOn=True
						odorOff=False
						lastOdorStartTime = time.time()

					if odorOn:
						mfc2_sp = (float(config.FLOW_RATE) / 666.0)*(float(config.PERCENT_AIR)/100.0) #air
						mfc3_sp = (float(config.FLOW_RATE) / 666.0)*(float(config.PERCENT_ACV)/100.0) #acv
					else:
						mfc2_sp = (float(config.FLOW_RATE) / 666.0)*(float(100)/100.0) #air
						mfc3_sp = 0.0 #acv

					print(odorOn, odorOff)
					SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(1,motorSendVal, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp) +'>\n'
					RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))

					logger.info('{} -- {}, {}, {}, {}, {}, {}, {}, {}, {}'.format((datetime.now()).strftime("%m/%d/%Y-%H:%M:%S.%f"),heading, posx,posy, motorSendVal, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp))

				if odorActivationCount > n_cycles:
					close_and_clean(RPI_SOCK,RPI_HOST, RPI_PORT)

		except KeyboardInterrupt:
			close_and_clean(RPI_SOCK,RPI_HOST, RPI_PORT)


if __name__=='__main__':
	odor_on_time, odor_off_time, n_cycles = parse_arguments()
	config.check_cfg()
	LOCAL_HOST, LOCAL_PORT = config.LOCAL_HOST, config.LOCAL_PORT
	RPI_HOST, RPI_PORT = config.RPI_HOST, config.RPI_PORT
	run_fictrac_client(LOCAL_HOST, LOCAL_PORT, RPI_HOST, RPI_PORT, odor_on_time, odor_off_time, n_cycles)
