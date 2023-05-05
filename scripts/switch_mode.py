import zmq
from time import sleep


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("ipc:///tmp/drone1/steer")

while(1):
    print('A')
    socket.send_string("m:acro")
    sleep(1)
    socket.send_string("m:angle")
    sleep(1)