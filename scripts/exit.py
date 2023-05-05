import zmq
from time import sleep


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("ipc:///tmp/drone1/steer")
while(1):
    socket.send_string("c:exit")
    sleep(1)