import socket
import threading

PI_IP = "192.168.50.159"   # Replace with your Pi’s IP
PI_PORT = 5006
MAC_PORT = 5005          # Mac listens here

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", MAC_PORT))

def receiver():
    print(f"Listening on port {MAC_PORT}...")
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received from {addr}: {data.decode()}")

def sender():
    while True:
        msg = input("Enter message to Pi: ")
        sock.sendto(msg.encode(), (PI_IP, PI_PORT))

#Run receive in background
threading.Thread(target=receiver, daemon=True).start()
sender()
