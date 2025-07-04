import socket
import time

# UDP broadcasting address and port
broadcast_ip = '255.255.255.255'
port = 5005

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Broadcast a message
message = b"Hello, devices!"
# while True:
#     sock.sendto(message, (broadcast_ip, port))
#     print("Broadcast message sent!")
#     time.sleep(1)  # Send every second

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5006))
sock.settimeout(1.0)  # Set timeout for recvfrom (in seconds)
print(f"Listening for broadcasts on port {port}...")
while True:
    try:
        sock.sendto(message, (broadcast_ip, port))
        print("Broadcast message sent!")
        data, addr = sock.recvfrom(4024)  # Buffer size is 1024 bytes
        print(f"Received message: {data} from {addr}")
        break
    except socket.timeout:
        print("No message received, waiting...")


data, addr = sock.recvfrom(4024)  # Buffer size is 1024 bytes
print(f"Received message: {len(data)} from {addr}")