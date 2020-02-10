import os
import logging
from datetime import datetime


def _check_for_log_dir():
	if 'logs' not in os.listdir(os.getcwd()):
		os.mkdir(os.path.join(os.getcwd(), 'logs'))

def init_logger():
	_check_for_log_dir()
	now = datetime.now()
	dts = now.strftime("%m%d%Y-%H%M%S")
	logger = logging.getLogger('closed_loop_output')
	hdlr = logging.FileHandler(os.path.join(os.getcwd(), 'logs','{}.log'.format(dts)))
	logger.addHandler(hdlr)
	logger.setLevel(logging.DEBUG)
	logger.info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format("timestamp", "motor_step_command","mfc1_stpt","mfc2_stpt","mfc3_stpt","led1_stpt", "led2_stpt","ft_posx","ft_posy","ft_heading","ft_frame","ft_error","ft_roll","ft_pitch","ft_yaw"))
	return logger
