import socket
import struct
from time import sleep


class UdpIkSender:
    def __init__(self):
        self.client_host = "192.168.1.10"
        self.client_port = 6013
        BUFFER_SIZE = 1024

        # 创建UDP套接字
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 绑定客户端地址和端口
        # client_socket.bind((client_host, client_port))

    def send(self, message):
        packed_data = b''.join([struct.pack('<f', num) for num in message])  # simulation >
        self.client_socket.sendto(packed_data, (self.client_host, self.client_port))

udp=UdpIkSender()
msg=[100,100,100,100,100,100,100]
while True:
    udp.send(msg)
    sleep(0.1)