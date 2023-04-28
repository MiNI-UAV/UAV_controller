from pynput.keyboard import Key, Listener, KeyCode
from time import sleep
import zmq


class State:
    Z = 0.0
    Fi = 0.0
    Theta = 0.0
    Psi = 0.0

    def __init__(self) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect("tcp://127.0.0.1:10001")

    def set(self,key: KeyCode):
        if('d' == key.char):
            self.Fi += 0.05
        if('a' == key.char):
            self.Fi -= 0.05
        if('s' == key.char):
            self.Theta += 0.05
        if('w' == key.char):
            self.Theta -= 0.05
        if('e' == key.char):
            self.Psi += 0.05
        if('q' == key.char):
            self.Psi -= 0.05
        if('z' == key.char):
            self.Z += 0.2
        if('x' == key.char):
            self.Z -= 0.2
        print(f"Z:{self.Z}, Fi:{self.Fi}, Theta:{self.Theta}, Psi:{self.Psi}")
        self.socket.send_string(f"Z:{self.Z},F:{self.Fi},T:{self.Theta},P:{self.Psi}")       


state = State()

with Listener(on_press = lambda key: state.set(key)) as listener:  
    listener.join()