
__author__ = "Alexander Wallar <aw204@st-andrews.ac.uk>"

__all__ = [
    "coordinator", "agent", "nameserver",
    "get_ns_host", "get_ns_port", "get_robot_name"
]

import coordinator
import agent
import nameserver
import os


NS_HOST_ID = "ZMQROS_NS_HOST"
NS_PORT_ID = "ZMQROS_NS_PORT"
ROBOT_ID = "ZMQROS_ROBOT_ID"


def get_ns_host():
    try:
        return os.environ[NS_HOST_ID]
    except KeyError:
        raise OSError("NS_HOST_ID not found")


def get_ns_port():
    try:
        return int(os.environ[NS_PORT_ID])
    except KeyError:
        raise OSError("NS_PORT_ID not found")
    except ValueError:
        raise OSError("NS_PORT_ID must be an integer")


def get_robot_name():
    try:
        return os.environ[ROBOT_ID]
    except KeyError:
        raise OSError("NS_PORT_ID not found")
