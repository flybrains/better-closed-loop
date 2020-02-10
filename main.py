import sys
import time
import queue
import random
import logging
import threading
import concurrent.futures
from datetime import datetime

import hardware_parameters as hw
from utilities import init_logger
from socket import socket, AF_INET, SOCK_STREAM, SOCK_DGRAM

class Interface(object):
	def __init__(self, hw_params=hw.params, type=None):
		self.local_host = hw_params['local_host']
		self.local_port = hw_params['local_port']
		self.type = type
		self.initialized = False

	def connect(self):
		if self.type is 'incoming':
			self.incoming_sock = socket(AF_INET, SOCK_STREAM)
			time.sleep(0.02)
			self.incoming_sock.connect((self.local_host, self.local_port))
			self.initialized = True
		elif self.type is 'outgoing':
			self.outgoing_sock = socket(AF_INET, SOCK_DGRAM)
			time.sleep(0.02)
		else:
			self.initialized = False

	def read(self):
		if self.type is 'incoming':
			self.data = self.incoming_sock.recv(1024)
			if not self.data:
				return False
			else:
				line = self.data.decode('UTF-8')
				self.toks = line.split(',')
				return True
		else:
			return False

	def kill(self):
		sock.close()

class FicTracInterface(Interface):
	def __init__(self):
		super().__init__(hw_params=hw.params, type='incoming')

	def clean(self):
		ft_frame, ft_error, ft_roll = float(self.data[1]), float(self.data[5]), float(self.data[6])
		ft_pitch, ft_yaw = float(self.data[7]), float(self.data[8])
		posx, posy = float(self.data[15])*3, -float(self.data[16])*3
		heading, net_vel = -float(self.data[17]),float(self.data[19])*3
		motor_heading_map = heading % (2*np.pi)
		prop_head = motor_heading_map/(2*np.pi)
		motor_target = int(prop_head*hw.params['steps_per_rev'])
		timestamp, mfc1, mfc2, mfc3, led1, led2 = None, None, None, None, None, None
		self.toks = [motor_target, heading, posx, posy, net_vel, ft_roll, ft_pitch, ft_yaw, ft_frame, ft_error, timestamp, mfc1, mfc2, mfc3, led1, led2]

class ReplayInterface(Interface):
	def __init__(self):
		super().__init__(hw_params=hw.params, type='incoming')

	def clean(self):
		# timestamp = self.data[0]
		# mfc1, mfc2, mfc3 = float(self.data[2]), float(self.data[3]), float(self.data[4])
		# led1, led2 = float(self.data[5]), float(self.data[6])
		# posx, posy, net_vel = float(self.data[7]), float(self.data[8]), float(self.data[9])
		# heading = float(self.data[10])
		# motor_target, ft_frame, ft_error, ft_roll, ft_pitch, ft_yaw = None, None, None, None, None, None
		# self.toks = [motor_target, heading, posx, posy, net_vel, ft_roll, ft_pitch, ft_yaw, ft_frame, ft_error, timestamp, mfc1, mfc2, mfc3, led1, led2]
		self.out = self.toks[0]

class RaspberryPiInterface(Interface):
	def __init__(self):
		super().__init__(hw_params=hw.params, type='outgoing')

	def write(self,toks):
		print('RPi Write',toks)

class LoggingInterface(Interface):
	def __init__(self):
		super().__init__(hw_params=hw.params, type='outgoing')
		self.logger = init_logger()

	def write(self,toks):
		toks.insert(0, datetime.now().strftime("%m/%d/%Y-%H:%M:%S.%f"))
		self.logger.info(','.join(toks))

class InterfaceManager(object):
	def __init__(self, use_fictrac=True, use_pi=True, use_logging=True):
		self.use_fictrac =  use_fictrac
		self.use_pi = use_pi
		self.use_logging = use_logging
		self.running = False
		self.outgoing_interfaces = []

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

	def run_interfaces(self, *args, **kwargs):
		if self.incoming_interface.initialized:
			self.running = True
			try:
				while self.running:
					ret = self.incoming_interface.read()
					self.incoming_interface.clean()
					if ret == False:
						break
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
	manager.run_interfaces(print=False)



	# outgoing_interface = RaspberryPiInterface()
	# outgoing_interface.connect_outgoing()
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
