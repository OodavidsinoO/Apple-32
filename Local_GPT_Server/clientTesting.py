from configs import HostIP, HostPort
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", HostPort))
print(" [✅] Connected to " + HostIP + ":" + str(HostPort) + "...")
print(" [🚨] Type QUIT to exit...")

while True:
    msg = input("User > ")
    while msg != "QUIT":
        s.send(msg.encode("utf-8"))
        while True: # Wait for response
            response = s.recv(1024).decode("utf-8")
            if response != "":
                break
        print("Apple One > " + response)
        msg = input("User > ")
    s.close()