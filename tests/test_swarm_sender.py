
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import zmqros
from zmqros.coordinator import open_swarm
import random
import time
from geometry_msgs.msg import Twist

ns_host = zmqros.get_ns_host()
ns_port = zmqros.get_ns_port()
nserver = zmqros.coordinator.NameServer(ns_host, ns_port)
names = nserver.get_names()


def run():
    with open_swarm(ns_host, ns_port, names) as swarm:
        while True:
            for bot in swarm.get_bots():
                t = Twist()
                t.linear.x = random.random()
                t.linear.y = random.random()
                t.linear.z = random.random()
                t.angular.x = random.random()
                t.angular.y = random.random()
                t.angular.z = random.random()

                bot.send_message("geometry_msgs/Twist", "/cmd_vel", t)

            time.sleep(0.1)

if __name__ == "__main__":
    run()
