#!/usr/bin/env python


import warnings
import rospy
import zmqros


def main():
    warnings.filterwarnings("ignore")
    rospy.init_node("zmqros_agent", anonymous=False, disable_signals=True)
    host = rospy.get_param("~ns_host", "localhost")
    port = rospy.get_param("~ns_port", 8000)
    name = rospy.get_param("~name")
    zmqros.agent.run(host, port, name)

if __name__ == "__main__":
    main()
