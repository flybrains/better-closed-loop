import queue
import threading
import hardware_parameters as hw

class Lights(object):
    def __init__(self, read_queue, write_queue):
        self.read_queue = read_queue
        self.write_queue = write_queue
        self.read = False
        self.written = False
        self.read_and_wrote = threading.Event()

    def _get_vals(self):
        try:
            self.read = True
            return self.read_queue.get()
        except UnboundLocalError:
            self.read = False
            return None

    def _put_vals(self, vals):
        try:
            self.write_queue.put()
            self.written = True
        except UnboundLocalError:
            self.written = False

    def _signal(self):
        if self.read and self.written:
            self.read_and_wrote.set()

    def _reset(self):
        # Look for signal from master
        self.read, self.written = False, False
        self.read_and_wrote.clear()


    def run(self, read_queue, write_queue):
        while 1:
            if self.read_and_wrote.is_set():
                pass
            else:
                vals_in = self._get_vals()

                # DO STUFF

                self._put_vals(vals_out)
                self._signal()
