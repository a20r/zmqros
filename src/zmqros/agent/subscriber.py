
import config
import threading
import json
import zmq
import signal
from util import create_addr


class Subscriber(object):

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sub = self.create_sub(host, port)
        self.running = False

    def create_sub(self, host, port):
        addr = create_addr(host, port)
        context = zmq.Context()
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.SUBSCRIBE, "")
        sub.connect(addr)
        return sub

    def live(self):
        while self.running:
            try:
                zmq_msg = self.sub.recv()
                zmq_dict = json.loads(zmq_msg)
                route_name = zmq_dict["route"]
                route_data = zmq_dict["data"]
                callback_list = config.callback_functions[route_name]
                for callback in callback_list:
                    callback(route_data)
            except zmq.ZMQError:
                self.running = False

    def start(self):
        self.running = True
        sub_thread = threading.Thread(target=self.live)
        sub_thread.daemon = True
        sub_thread.start()
        return sub_thread

    def kill(self):
        self.running = False
