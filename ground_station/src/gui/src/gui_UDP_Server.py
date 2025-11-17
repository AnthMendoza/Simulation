import time
import socket
import threading


class server:
    def __init__(self, host="127.0.0.1", port=5050, UDP_callback=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.sock.bind((host,port))

        self.sock.settimeout(0.1)

        self.running = False
        self.thread = None

        self.UDP_callback = UDP_callback
        
        self.bufferSize = 1024

    def start(self):
        self.running = True
        self.thread = threading.Thread(target = self.poll ,daemon=True)
        self.thread.start()
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.sock.close()
    
    def poll(self):
        while self.running:
            try:
                data , addr = self.sock.recvfrom(self.bufferSize)
                self.UDP_callback(data)
            except socket.timeout:
                continue

















