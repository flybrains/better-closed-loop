import sys
import os
import socket
import time
import config
from datetime import datetime

class Replayer(object):
    def __init__(self, log_files, host, port, flush_duration):
        self.log_files = log_files
        self.connected = False
        self.host = host
        self.port = port
        self.flushed = True
        self.flush_duration = int(flush_duration)

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

    def _replay(self, log_address):
        playback, times = self._parse_log(log_address)
        if self.flushed:
            self.flushed = False
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.sock:
                self._connect()

                with self.conn:
                    self.conn.send(str.encode('{},{},{}'.format(playback[0][0],playback[0][1],playback[0][2])))
                    time.sleep(0.015)
                    index = 0
                    while True:
                        try:
                            st = str(times[index+1] - times[index]).split('.')
                            if len(st)==1:
                                delta=float(0.0)
                            else:
                                delta = float('0.{}'.format(st[-1]))
                            time.sleep(delta)
                            print('{},{},{}'.format(playback[index][0],playback[index][1],playback[index][2]))
                            self.conn.send(str.encode('{},{},{}'.format(playback[index][0],playback[index][1],playback[index][2])))
                            index += 1
                        except IndexError:
                            return None
        else:
            self._flush(self.flushDur)

    def _connect(self):
        self.sock.bind((self.host, self.port))
        self.sock.listen()
        self.conn, self.addr = self.sock.accept()

    def _flush(self):
        self.flushed=True
        with self.conn:
            self.conn.send(str.encode('{},{},{}'.format(0,0.99,0)))
            time.sleep(int(self.flush_duration*60/2))
            self.conn.send(str.encode('{},{},{}'.format(0,0,0)))
            time.sleep(int(self.flush_duration*60/2))
            self.flushed = True

    def run_batch(self):
        for file in self.log_files:
            self._replay(file)
            self._flush()


if __name__=='__main__':
    HOST, PORT = config.LOCAL_HOST, config.LOCAL_PORT
    log_files = [os.path.join(config.REPLAY_FOLDER,e) for e in os.listdir(config.REPLAY_FOLDER)]
    replayer = Replayer(log_files, HOST, PORT, flush_duration=0)
    replayer.run_batch()
