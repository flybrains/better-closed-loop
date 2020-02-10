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
		# timestamp = self.toks[0]
		# mfc1, mfc2, mfc3 = float(self.toks[2]), float(self.toks[3]), float(self.toks[4])
		# led1, led2 = float(self.toks[5]), float(self.toks[6])
		# posx, posy, net_vel = float(self.toks[7]), float(self.toks[8]), float(self.toks[9])
		# heading = float(self.toks[10])
		# motor_target, ft_frame, ft_error, ft_roll, ft_pitch, ft_yaw = None, None, None, None, None, None
		# self.toks = [motor_target, heading, posx, posy, net_vel, ft_roll, ft_pitch, ft_yaw, ft_frame, ft_error, timestamp, mfc1, mfc2, mfc3, led1, led2]
		return self.toks[0]

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
