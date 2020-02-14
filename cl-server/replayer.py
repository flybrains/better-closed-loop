import sys
import os
import socket
import time
import threading
import numpy as np
from datetime import datetime
import hardware_parameters as hw

class Replayer(object):
    def __init__(self, shutdown):#self, log_files):
        # self.log_files = log_files
        # self.connected = False
        self.host = hw.params['source_host']
        self.port = hw.params['source_port']
        self.shutdown = shutdown


    @staticmethod
    def _parse_log(address):
        playback = []
        times = []
        with open(address) as f:
            for idx, row in enumerate(f.read().split("\n")):
                if idx==0:
                    pass
                else:
                    try:
                        time, toks = row.split(" -- ")[0], row.split(" -- ")[1]
                        time = time.split("-")[1]

                        dt = datetime.strptime(time, '%H:%M:%S.%f')
                        times.append(dt)
                        toks = [e.strip() for e  in toks.split(',')]
                        # mfc1, mfc2, mfc3
                        playback.append([float(toks[1]),float(toks[2]),float(toks[3])])
                    except IndexError:
                        pass
        return playback, times

    def replay(self):#, log_address):
        #playback, times = self._parse_log(log_address)
        with self.conn:
            #self.conn.send(str.encode('0'))
            #self.conn.send(str.encode('{},{},{}'.format(playback[0][0],playback[0][1],playback[0][2])))
            time.sleep(0.015)
            index = 0
            for i in range(20000):
            #while True:

                try:
                    if self.shutdown.is_set():
                        break
                    #print(i)

                    # print('================')
                    # print(i)
                    delta = float(1/30)
                    # st = str(times[index+1] - times[index]).split('.')
                    # if len(st)==1:
                    #     delta=float(0.0)
                    # else:
                    #     delta = float('0.{}'.format(st[-1]))
                    time.sleep(delta)
                    self.conn.send(str.encode('{}'.format(i)))#self.conn.send(str.encode('{},{},{}'.format(np.random.randint(1,100),np.random.randint(1,100),np.random.randint(1,100))))
                    # self.conn.send(str.encode('{},{},{}'.format(playback[index][0],playback[index][1],playback[index][2])))
                    index += 1

                except (KeyboardInterrupt, BrokenPipeError, ConnectionResetError) as e:
                    self.sock.close()
                    sys.exit()


    def bind(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen()
        self.conn, self.addr = self.sock.accept()


if __name__=='__main__':
    # HOST, PORT = config.LOCAL_HOST, config.LOCAL_PORT
    # log_files = [os.path.join(config.REPLAY_FOLDER,e) for e in os.listdir(config.REPLAY_FOLDER)]
    replayer = Replayer('127.0.0.1', 3000)#log_files, HOST, PORT, flush_duration=0)
    #replayer.run_batch()
    replayer._replay()
