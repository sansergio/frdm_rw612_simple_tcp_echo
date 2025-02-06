import socket

HOST = '192.168.0.102'  # The server's hostname or IP address
PORT = 57777        # The port used by the server

try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print("Connected!")
        while True:
            message = input()
            s.sendall(message.encode())
            data = s.recv(1024)
            print('Received', repr(data.decode()))
except ConnectionRefusedError:
    print("Connection failed!")