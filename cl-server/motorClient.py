import time
import socket
import hardware_parameters as hw

class MotorClient(object):
    def __init__(self):
        self.host = "127.0.0.1"
        self.port = 8080

    def connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.host, self.port))
        self.client.sendall(bytes("Motor Connected",'UTF-8'))

    def run(self):
        while True:
            data = self.client.recv(1024).decode()
            if data=='<>':
                self.client.sendall(bytes('<>'.format(data),'UTF-8'))
                self.client.close()
                break
            data = int(data)*1
            self.client.sendall(bytes('<mt_{}>'.format(data),'UTF-8'))
