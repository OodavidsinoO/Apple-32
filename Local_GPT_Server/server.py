# Author: David Sin
# Date: 2023-11-24
# ELEC 3300 Local GPT Server for Apple One Emulator
from configs import system_template, prompt_template, modelName, HostIP, HostPort

from pathlib import Path
from gpt4all import GPT4All
import socket

# Create socket server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HostIP, HostPort))
s.listen()
print(" [😎] Author: David Sin")
print(" [😎] ELECT 3300 Local GPT Server for Apple One Emulator")
print(" [✅] TCP Server listening as " + HostIP + ":" + str(HostPort) + "...")
print(" [✅] Model: " + modelName)

# Load model
model = GPT4All(model_name = modelName,
                model_path = Path(__file__).parent / "models",
                allow_download = True,
                verbose = True)

# Model chat session
with model.chat_session(system_template, prompt_template):
    response = model.generate("Hello")
    print(" [✅] Model chat session started...")
    print(" [✅] Waiting for client connection...")
    # Accept client connection
    while True:
        clientsocket, address = s.accept()
        print(" [✅] Client connected: " + str(address))
        # Receive client message
        while True:
            msg = clientsocket.recv(1024).decode("utf-8").lower()
            if msg == "quit":
                # End chat session
                print(" [⛔] Model chat session ended...")
                print(" [⛔] TCP Server stopped...")
                s.close()
                exit()
            # Generate model response
            if msg != "":
                print(" [📩] " + str(address) + " > " + msg)
                response = model.generate(msg)
                print(" [📤] " + str(address) + " > " + response)
                clientsocket.send(response.encode("utf-8"))