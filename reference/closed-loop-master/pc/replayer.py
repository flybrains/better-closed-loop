import sys
import socket
import time
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
                        # posx, posy, mfc1, mfc2, mfc3
                        playback.append([float(toks[1]),float(toks[2]),float(toks[3]),float(toks[6]),float(toks[7])])
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
                    self.conn.send(b'{},{},{},{},{}'.format(playback[0][0],playback[0][1],playback[0][2],playback[0][3],playback[0][4]))
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
                            self.conn.send(b'{},{},{},{},{}'.format(playback[index][0],playback[index][1],playback[index][2],playback[index][3],playback[index][4]))
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
            self.conn.send(b'{},{},{},{},{}'.format(0,0,0,0.99,0))
            time.sleep(int(self.flush_duration*60/2))
            self.conn.send(b'{},{},{},{},{}'.format(0,0,0,0,0))
            time.sleep(int(self.flush_duration*60/2))
            self.flushed = True

    def run_batch(self):
        for file in self.log_files:
            self._replay(file)
            self._flush()


if __name__=='__main__':
    # '127.0.0.1', 3100
    HOST, PORT = str(sys.argv[1]), int(sys.argv[2])
    log_files = [str(e) for e in sys.argv[3:]]
    print(HOST, PORT, log_files)
    replayer = Replayer(log_files, HOST, PORT, flush_duration=0)
    replayer.run_batch()
