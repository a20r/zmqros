#!/usr/bin/env python


import warnings
import rospy
import zmqros
import signal
import sys


def signal_handler(signal, frame):
    sys.exit(0)


def main():
    warnings.filterwarnings("ignore")
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("zmqros_ns", anonymous=False)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 8000)
    zmqros.nameserver.run(host, port)

if __name__ == "__main__":
    main()
