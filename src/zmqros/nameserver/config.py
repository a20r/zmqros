
from flask import Flask
import collections

Connection_ = collections.namedtuple("Connection", "host port")


class Connection(Connection_):
    def __hash__(self):
        return hash(self.host + str(self.port))


app = Flask(__name__)
app.config.from_object(__name__)

#  Stores the robots that are currently sending heartbeat messages
live_robots = dict()

#  Dictionary from hostname to list of allocated ports for ZeroMQ
available_ports = dict()

#  Dictionary of robot connections
robot_connections = dict()

#  Dictionary of new robot connections
new_connections = dict()

#  Minimum port to be allocated per hostname
min_port = 5555

#  Maximum port to be allocated per hostname
max_port = 6555

#  Acceptable heartbeat delay time
heartbeat_delay = 5  # seconds
