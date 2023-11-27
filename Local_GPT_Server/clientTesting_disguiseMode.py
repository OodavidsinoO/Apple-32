from configs import HostIP, HostPort
from os import system
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", HostPort))
system("cls")
system("title COM7: Apple I Serial Terminal")
print("Press <space> to boot Apple I")
print("Initializing CPU... Complete")
print("Initializing RAM... Complete")
print("Initializing PIA... Complete")

while True:
    msg = input("\\\n")
    while msg != "QUIT" or msg != "quit":
        s.send(msg.encode("utf-8"))
        while True: # Wait for response
            response = s.recv(1024).decode("utf-8")
            if response != "":
                break
        print("Apple One:\n" + response)
        msg = input("\\\n")
    s.close()