import sys
import time
import queue
import socket
import threading
import numpy as np
import concurrent.futures
import hardware_parameters as hw

from motorClient import MotorClient
from lightClient import LightClient
from mfcClient import MFCClient
from replayer import Replayer


class ClientManagerThread(threading.Thread):
    def __init__(self,clientAddress, clientsocket, data_queue, new_source_data, read_and_wrote, shutdown, client_shutdown):
        threading.Thread.__init__(self)
        self.csocket = clientsocket
        self.clientAddress = clientAddress
        self.source_data_queue = data_queue[0]
        self.destination_data_queue = data_queue[1]
        self.new_source_data = new_source_data
        self.read_and_wrote = read_and_wrote
        self.shutdown = shutdown
        self.client_shutdown = client_shutdown

    def _get_new_data(self):
        return self.source_data_queue.get()

    def _put_new_data(self, val):
        self.destination_data_queue.put(val)

    def run(self):
        while not self.shutdown.is_set():
            self.new_source_data.wait()
            new_source_data = self._get_new_data()
            if self.client_shutdown.is_set():
                new_source_data = '<>'
            self.csocket.send(bytes('{}'.format(new_source_data),'UTF-8'))
            data = self.csocket.recv(2048).decode()
            if data=='<>':
                self._put_new_data(data)
                self.read_and_wrote.set()
                break
            self._put_new_data(data)
            self.read_and_wrote.set()
            self.new_source_data.clear()
        print('Client Thread = Safe Exit')



class Server(object):
    def __init__(self, n_clients, hw_params=hw.params, type=None):
        self.local_host = hw_params['local_host']
        self.local_port = hw_params['local_port']
        self.source_host = hw_params['source_host']
        self.source_port = hw_params['source_port']

        self.queue_pairs = []
        self.read_and_wrote_events = []
        self.shutdown = threading.Event()
        self.client_shutdown = threading.Event()
        self.new_source_data = threading.Event()
        self.read_next_from_source = threading.Event()
        self.read_next_from_source.set()
        self.n_clients = n_clients
        
    def connect_source(self):
        # Add server as client to source such as Fictrac or replayer
        self.source_reader = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.source_reader.connect((self.source_host, self.source_port))

    def bind(self):
        # Bind server to localhost
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.local_host, self.local_port))

    def activate_shutdown(self, mode):
        if mode=='soft':
            self.client_shutdown.set()

        if mode=='hard':
            self.shutdown.set()
            self.read_next_from_source.set()

    def read_from_source(self):
        while not self.shutdown.is_set():

            self.read_next_from_source.wait()
            self.from_source = self.source_reader.recv(1024)
            data = self.from_source.decode('UTF-8')
            [q[0].put(data) for q in self.queue_pairs]
            self.new_source_data.set()
            self.read_next_from_source.clear()

            # if not self.read_next_from_source.isSet():
            #     # If not ready, read and dump buffer to prevent backup
            #     self.from_source = self.source_reader.recv(1024)
            # else:
            #     # If ready, read buffer and add to client queues
            #     self.from_source = self.source_reader.recv(1024)
            #     data = self.from_source.decode('UTF-8')
            #     [q[0].put(data) for q in self.queue_pairs]
            #     self.new_source_data.set()
            #     self.read_next_from_source.clear()


        print('Server Read Thread = Safe Exit')


    def write_to_destination(self):
        a = time.time()
        while not self.shutdown.is_set():
            # Wait for all client threads to finish read/write cycle
            [rwe.wait() for rwe in self.read_and_wrote_events]
            from_clients = [q[1].get() for q in self.queue_pairs]

            if from_clients[0]=="<>":
                break
            print('=============================')
            print(from_clients)
            print("Latency: ", int(float(1000000*(time.time()-a - float(1/30)))), "us")
            a = time.time()
            #########################
            # Write to Pi here
            #########################
            self.read_next_from_source.set()
            [rwe.clear() for rwe in self.read_and_wrote_events]

        print('Server Write Thread = Safe Exit')


    def bind_clients(self):

        for i in range(self.n_clients):
            self.server.listen(1)
            clientsock, clientAddress = self.server.accept()

            queue_pair = [queue.Queue(), queue.Queue()]
            self.queue_pairs.append(queue_pair)

            read_and_wrote = threading.Event()
            self.read_and_wrote_events.append(read_and_wrote)

            newthread = ClientManagerThread(clientAddress, clientsock, queue_pair, self.new_source_data, read_and_wrote, self.shutdown, self.client_shutdown)
            newthread.start()

    def main(self):
        self.clients = [LightClient(), MotorClient(), MFCClient()]

        self.replayer = Replayer(self.shutdown)

        self.replay_binder = threading.Thread(target=self.replayer.bind)
        self.replay_binder.start()
        self.connect_source()
        self.replay_binder.join()

        self.bind()
        self.clients_binder = threading.Thread(target=self.bind_clients)
        self.clients_binder.start()
        [client.connect() for client in self.clients]
        self.clients_binder.join()

        self.replay_thread = threading.Thread(target=self.replayer.replay)
        self.replay_thread.start()
        self.reader_thread = threading.Thread(target=self.read_from_source)
        self.reader_thread.start()

        self.client_threads = [threading.Thread(target=client.run) for client in self.clients]
        [thread.start() for thread in self.client_threads]

        self.writer_thread = threading.Thread(target=self.write_to_destination)
        self.writer_thread.start()

        input()
        self.activate_shutdown('soft')
        [thread.join() for thread in self.client_threads]
        self.activate_shutdown('hard')

        self.reader_thread.join()
        self.writer_thread.join()
        self.replay_thread.join()
        print('Done... Shutting Down')

if __name__=='__main__':
    server = Server(3)
    server.main()

































#
