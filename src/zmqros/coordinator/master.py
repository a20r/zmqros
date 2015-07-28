
import json
import zmq
from rospy_message_converter import message_converter


class Master(object):

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.addr = self.create_addr(self.host, self.port)
        self.init_pub()

    def init_pub(self):
        ctx = zmq.Context()
        self.pub = ctx.socket(zmq.PUB)
        self.pub.bind(self.addr)

    def send_message(self, msg_type, topic_name, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        send_dict = dict()
        data_dict = dict()
        send_dict["route"] = "topic"
        data_dict["topic_name"] = topic_name
        data_dict["msg_type"] = msg_type
        data_dict["msg"] = msg_dict
        send_dict["data"] = data_dict
        json_msg = json.dumps(send_dict)
        self.pub.send(zmq.Message(json_msg))
        return send_dict

    def create_addr(self, host, port):
        return "tcp://{}:{}".format(host, port)
