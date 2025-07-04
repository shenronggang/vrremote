import socket
import struct


class UdpIkSender:
    def __init__(self,host="127.0.0.1",port=5005):
        self.client_host = host
        self.client_port = port
        BUFFER_SIZE = 1024

        # 创建UDP套接字
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 绑定客户端地址和端口
        # client_socket.bind((client_host, client_port))

    def send(self, message):
        packed_data = b''.join([struct.pack('<f', num) for num in message])  # simulation >
        self.client_socket.sendto(packed_data, (self.client_host, self.client_port))
        # #print('send')