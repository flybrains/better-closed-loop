import sys
import time
import queue
import random
import logging
import threading
import concurrent.futures
from datetime import datetime

from lights import Lights

import hardware_parameters as hw
from utilities import init_logger
from socket import socket, AF_INET, SOCK_STREAM, SOCK_DGRAM



class InterfaceManager(object):
	def __init__(self, use_fictrac=True, use_pi=True, use_logging=True):
		self.use_fictrac =  use_fictrac
		self.use_pi = use_pi
		self.use_logging = use_logging
		self.running = False
		self.outgoing_interfaces = []

		self.light_input_queue = queue.Queue()
		self.light_output_queue = queue.Queue()


	def _spin_incoming_interface(self):
		if self.use_fictrac:
			self.incoming_interface = FicTracInterface()
		else:
			self.incoming_interface = ReplayInterface()
		self.incoming_interface.connect()

	def _spin_outgoing_interfaces(self):
		if self.use_pi:
			self.outgoing_interfaces.append(RaspberryPiInterface())
		if self.use_logging:
			self.outgoing_interfaces.append(LoggingInterface())

		[interface.connect() for interface in self.outgoing_interfaces]

	def _spin_hardware_channels(self):
		self.lights = Lights(self.light_input_queue, self.light_output_queue)
		self.motor = Lights(self.light_input_queue, self.light_output_queue)
		self.mfcs = Lights(self.light_input_queue, self.light_output_queue)

	def run_interfaces(self):

		if self.incoming_interface.initialized == False:
			sys.exit()

		self.running = True
		a = time.time()
		try:
			while self.running:
				ret = self.incoming_interface.read()
				output = self.incoming_interface.clean()
				if ret == False:
					break

				self.light_input_queue.put([output[i] for i in self.lights.selection_mask])

				self.lights.read_and_wrote.wait()
				updated = self.light_output_queue.get()
				self.lights.reset()

				[interface.write(data) for interface in self.outgoing_interfaces]

		except KeyboardInterrupt:
			sys.exit()

	def safe_shutdown(self):
		light_controller.safe_shutdown()
		motor_controller.safe_shutdown()
		mfc_controller.safe_shutdown()


if __name__=='__main__':
	manager = InterfaceManager(use_fictrac=False, use_pi=False, use_logging=False)
	manager._spin_incoming_interface()
	manager._spin_outgoing_interfaces()
	manager._spin_hardware_channels()

	lightKill = threading.Event()
	mainKill = threading.Event()

	with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
		executor.submit(manager.run_interfaces)
		executor.submit(manager.lights.run)


	#
	# light_input = queue.Queue(maxsize=30)
	# # motor_input = queue.queue()
	# # mfc_input = queue.queue()
	#
	# light_output = queue.queue()
	# motor_output = queue.queue()
	# mfc_output = queue.queue()

	# light_controller = Lights()
	# motor_controller = Motor()
	# mfc_controller = MFC()

	# with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
	# 	executor.submit(light_controller.run, light_input, light_output)
	# 	executor.submit(motor_controller.run, motor_input, motor_output)
	# 	executor.submit(mfc_controller.run, mfc_input, mfc_output)
	#
	# 	while True:
	# 		ret = incoming_interface.read_incoming()
	# 		if not ret:
	# 			safe_shutdown()
	# 			break
	# 		data = incoming_interface.clean()
	# 		light_info, motor_info, mfc_info = break_for_subsystems(data)
	# 		light_input.put(light_info)
	# 		motor_input.put(motor_info)
	# 		mfc_input.put(mfc_info)
	#
	# 		while not light_controller.read_and_wrote.is_set() and not motor_controller.read_and_wrote.is_set() and not mfc_controller.read_and_wrote.is_set():
	# 			time.sleep(0.01)
	#
	# 		light_controller.reset()
	# 		motor_controller.reset()
	# 		mfc_controller.reset()
	#
	# 		a = light_output.get()
	# 		b = motor_output.get()
	# 		c = mfc_output.get()
	# 		out_data= join(a,b,c)
	# 		send_to_pi(out_data)
	#
	# 	safe_shutdown()
	#
	# incoming_interface.kill()
	# outgoing_interface.kill()















#
#
# def run_fictrac_client(LOCAL_HOST, LOCAL_PORT, RPI_HOST, RPI_PORT):
#
#
# 	with socket(AF_INET, SOCK_STREAM) as sock:
# 		time.sleep(0.02)
# 		sock.connect((LOCAL_HOST, LOCAL_PORT))
#
# 		mfc1_sp, mfc2_sp, mfc3_sp = 0.0, 0.0, 0.0
# 		led1_sp, led2_sp = 0.0, 0.0
# 		motorSendVal = 800000
# 		lastMotorSendVal = motorSendVal
# 		expStartTime = time.time()
# 		LEDLastTime = expStartTime
# 		odorLastTime = expStartTime
# 		motorLastTime = expStartTime
# 		stimLastTime = expStartTime
# 		slidingWindow = [[0,0,0,0],[0,0,0,0]]
# 		activating=False
# 		stim = False
# 		previousAngle = 800000
# 		mult = 1000
#
# 		try:
# 			i = 0
# 			while True:
# 				data = sock.recv(1024)
# 				if not data:
# 					break
# 				line = data.decode('UTF-8')
# 				toks = line.split(',')
#
# 				if config.REPLAY:
# 					# posx, posy, mfc1, mfc2, mfc3
# 					mfc1, mfc2, mfc3 = toks[0], toks[1],toks[2]
# 					SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(1,800000, mfc1, mfc2, mfc3, 0.0, 0.0) +'>\n'
# 					RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))
#
# 				else:
# 					if ((len(toks) < 24) | (toks[0] != "FT")):
# 						('Bad read')
# 						continue
# 					ft_frame = int(toks[1])
# 					ft_error = float(toks[5])
# 					ft_roll = float(toks[6])
# 					ft_pitch = float(toks[7])
# 					ft_yaw = float(toks[8])
# 					posx = float(toks[15])*3
# 					posy = -float(toks[16])*3
# 					heading = float(toks[17])
# 					net_vel = float(toks[19])*3
#
# 					ft_heading = heading
# 					heading = -heading
# 					motorHeadingMap = heading % (2*np.pi)
# 					propHead = motorHeadingMap/(2*np.pi)
# 					target = int(propHead*800)
# #
# 					if len(slidingWindow) >= config.SLIDING_WINDOW_LENGTH:
# 						slidingWindow.pop(0)
# 					slidingWindow.append([posx, posy, net_vel, heading])
#
# 					if (time.time() - motorLastTime) > (1/60000):
#
# 						correctedTarget, mult = convert_angle_for_arduino(target, previousAngle, mult)
#
# 						previousAngle, motorSendVal = correctedTarget, correctedTarget
# 						motorLastTime = time.time()
#
# 						if config.PROPORTIONAL_ODOR==False:
#
# 							mfc3_sp = (float(config.FLOW_RATE) / 666.0)*(float(config.PERCENT_ACV)/100.0) #acv
# 							mfc2_sp = 0.0 #empty
# 							mfc1_sp = (float(config.FLOW_RATE) / 666.0)*(float(config.PERCENT_AIR)/100.0)	#air
#
#
# 						else:
# 							if config.REPEATING_STRIPS:
# 								if np.cos(np.pi*posx/config.XON) < 0:
# 									print('(x, y) - (', np.round(posx),',',np.round(posy), ') -- outside of strip')
# 									mfc3_sp = float(0.1*config.MAX_TOTAL_AIRFLOW)/666.0
# 									mfc2_sp = 0.0
# 									mfc1_sp = float(0.9*config.MAX_TOTAL_AIRFLOW)/666.0
# 								elif posy >= config.SINGLE_SOURCE_DISTANCE:
# 									print('(x, y) - (', np.round(posx),',',np.round(posy), ') -- constant odor')
# 									mfc3_sp = float(config.MAX_TOTAL_AIRFLOW)/666.0
# 									mfc2_sp = 0.0
# 									mfc1_sp = 0.0
#
#
# 									if config.REINFORCEMENT_AT_END > 0:
#
# 										if config.LED_COLOR=='red':
# 											led1_sp = config.LED_INTENSITY
# 											led2_sp = 0.0
# 										else:
# 											led1_sp = 0.0
# 											led2_sp = config.LED_INTENSITY
#
# 										SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(1,motorSendVal, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp) +'>\n'
# 										RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))
# 										now = datetime.now()
# 										dts = now.strftime("%m/%d/%Y-%H:%M:%S.%f")
# 										logger.info("{} -- {},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(dts, correctedTarget, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp, posx, posy, net_vel,ft_heading, ft_frame, ft_error, ft_roll, ft_pitch, ft_yaw))
#
# 										time.sleep(config.REINFORCEMENT_AT_END)
#
# 										SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(0,motorSendVal, 0, 0, 0, 0, 0) +'>\n'
# 										RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))
# 										now = datetime.now()
# 										sys.exit()
#
# 								else:
# 									print('(x, y) - (', np.round(posx),',',np.round(posy), ') -- in gradient')
# 									mfc1_sp = (float(config.MAX_TOTAL_AIRFLOW) / 666.0)*float(1-(posy/config.SINGLE_SOURCE_DISTANCE))
# 									mfc1_sp = max(0, mfc1_sp)
# 									mfc2_sp = 0.0
# 									mfc3_sp = (float(config.MAX_TOTAL_AIRFLOW) / 666.0)*float(posy/config.SINGLE_SOURCE_DISTANCE)
# 									acv_min = float(0.1*config.MAX_TOTAL_AIRFLOW)/666.0
# 									mfc3_sp = max(acv_min, mfc3_sp)
#
# 							elif config.N_ODOR_SOURCES == 1:
# 								if posy < 0:
# 									mfc3_sp = float(0.1*config.MAX_TOTAL_AIRFLOW)/666.0
# 									mfc2_sp = 0.0
# 									mfc1_sp = float(0.9*config.MAX_TOTAL_AIRFLOW)/666.0
#
#
# 									if config.REINFORCEMENT_AT_END > 0:
#
# 										if config.LED_COLOR=='red':
# 											led1_sp = config.LED_INTENSITY
# 											led2_sp = 0.0
# 										else:
# 											led1_sp = 0.0
# 											led2_sp = config.LED_INTENSITY
#
# 										SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(1,motorSendVal, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp) +'>\n'
# 										RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))
# 										now = datetime.now()
# 										dts = now.strftime("%m/%d/%Y-%H:%M:%S.%f")
# 										logger.info("{} -- {},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(dts, correctedTarget, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp, posx, posy, net_vel,ft_heading, ft_frame, ft_error, ft_roll, ft_pitch, ft_yaw))
# 										time.sleep(config.REINFORCEMENT_AT_END)
# 										SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(0,motorSendVal, 0, 0, 0, 0, 0) +'>\n'
# 										RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))
# 										now = datetime.now()
# 										sys.exit()
#
# 								else:
# 									mfc1_sp = (float(config.MAX_TOTAL_AIRFLOW) / 666.0)*float(1-(posy/config.SINGLE_SOURCE_DISTANCE))
# 									mfc1_sp = max(0, mfc1_sp)
# 									mfc2_sp = 0
# 									mfc3_sp = (float(config.MAX_TOTAL_AIRFLOW) / 666.0)*float(posy/config.SINGLE_SOURCE_DISTANCE)
# 									acv_min = float(0.1*config.MAX_TOTAL_AIRFLOW)/666.0
# 									mfc3_sp = max(acv_min, mfc3_sp)
#
# 						if config.LED_ACTIVATION:
# 							if config.STIMULATION_MODE == 't':
# 								if (time.time() - expStartTime) >= config.INITIAL_LED_DELAY:
# 									if ((time.time() - LEDLastTime) >= config.PERIOD_BETWEEN_PULSE_RISING_EDGES):
# 										if activating==False:
# 											print('LIGHTS ON')
# 											activating = True
# 											pulseStarted = time.time()
#
# 											if config.LED_COLOR=='red':
# 												led1_sp = config.LED_INTENSITY
# 												led2_sp = 0.0
# 											else:
# 												led1_sp = 0.0
# 												led2_sp = config.LED_INTENSITY
#
# 										elif activating==True and (time.time()-pulseStarted)<=config.DURATION_OF_PULSE_T:
# 											pass
# 										else:
# 											activating=False
# 											led1_sp = 0.0
# 											led2_sp = 0.0
# 											LEDLastTime = pulseStarted
#
# 							if config.STIMULATION_MODE == 'c':
# 								if config.CONDITION_TO_USE==1:
# 									d = np.diff([i[1] for i in slidingWindow])
# 									da = np.mean(d)
# 									if da >= config.CONDITION_THRESHOLD:
# 										stim=True
# 								# ++++++++++++++++++++++++++++++++++++
# 								# ADD MORE KINEMATIC CONDITIONS HERE
# 								# ++++++++++++++++++++++++++++++++++++
# 								if stim and (time.time() - stimLastTime)>=config.LOCKOUT_TIME_AFTER_PULSE:
# 									if activating==False:
# 										activating = True
# 										pulseStarted = time.time()
#
# 										if config.LED_COLOR=='red':
# 											led1_sp = config.LED_INTENSITY
# 											led2_sp = 0.0
# 										else:
# 											led1_sp = 0.0
# 											led2_sp = config.LED_INTENSITY
#
# 									elif activating and (time.time()-pulseStarted)<=config.DURATION_OF_PULSE_C:
# 										pass
# 									else:
# 										activating = False
# 										stim = False
# 										stimLastTime = pulseStarted
# 										led1_sp = 0.0
# 										led2_sp = 0.0
#
# 						else:
# 							led1_sp = 0.0
# 							led2_sp = 0.0
# 						SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(1,motorSendVal, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp) +'>\n'
# 						RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), ("192.168.137.10", 5000))
# 						now = datetime.now()
# 						dts = now.strftime("%m/%d/%Y-%H:%M:%S.%f")
# 						logger.info("{} -- {},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(dts, correctedTarget, mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp, posx, posy, net_vel,ft_heading, ft_frame, ft_error, ft_roll, ft_pitch, ft_yaw))
#
#
# 		except KeyboardInterrupt:
# 			SENDSTRING = '<'+ '{},{},{},{},{},{},{}'.format(0, 'a', mfc1_sp, mfc2_sp, mfc3_sp, led1_sp, led2_sp) +'>\n'
# 			RPI_SOCK.sendto(str.encode('{}'.format(SENDSTRING)), (RPI_HOST, RPI_PORT))
# 			RPI_SOCK.close()
