import time
import socket
import hardware_parameters as hw



class LightClient(object):
    def __init__(self):
        self.host = "127.0.0.1"
        self.port = 8080

    def _connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.host, self.port))
        self.client.sendall(bytes("This is from Client",'UTF-8'))

    def run(self):
        while True:
            data = self.client.recv(1024)

            print("From Server :" ,data.decode())

            c = 'Light Stuff'
            out_data = '{}'.format(c)
            self.client.sendall(bytes(out_data,'UTF-8'))

            if out_data=='_':
                break

        self.client.close()

if __name__=='__main__':
    lc = LightClient()
    lc._connect()
    lc.run()
