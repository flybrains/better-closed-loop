import time
import queue
import threading
import hardware_parameters as hw

class Lights(object):
    def __init__(self, read_queue, write_queue):
        self.selection_mask = [0]#[1,2,3,4,14,15]
        self.running = True
        self.read = False
        self.written = False
        self.read_and_wrote = threading.Event()
        self.read_queue = read_queue
        self.write_queue = write_queue

    def _get_vals(self):
        self.read = True
        return self.read_queue.get()

    def _put_vals(self, vals):
        self.write_queue.put(vals)
        self.written = True

    def _signal(self):
        if self.read and self.written:
            self.read_and_wrote.set()

    def reset(self):
        self.read, self.written = False, False
        self.read_and_wrote.clear()

    def safe_shutdown(self):
        self.running = False

    def run(self):

        while self.running:
            if self.read_and_wrote.is_set():
                pass
            else:
                vals_in = self._get_vals()

                # Logic Here
                vals_out = int(vals_in[0])*4

                self._put_vals(vals_out)
                self._signal()
