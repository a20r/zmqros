
import config
import util
import rospy
from rospy_message_converter import message_converter


def create_msg(msg_dict, msg_type):
    namespace, msg_name = msg_type.split("/")
    mod = __import__(namespace + ".msg")
    msg_cls = getattr(mod.msg, msg_name)

    msg = message_converter.convert_dictionary_to_ros_message(
        msg_type, msg_dict
    )

    return msg, msg_cls


@util.route("topic")
def post_topic(form):
    """
    Input:
        {
            "topic_name": <String>,
            "msg_type": <String: Namespaces delimited by `/`>,
            "msg": <JSON Dictionary>
        }

    """

    topic_name = form["topic_name"]
    msg, msg_cls = create_msg(form["msg"], form["msg_type"])

    if not topic_name in config.publishers:
        ros_pub = rospy.Publisher(
            topic_name, msg_cls
        )
        config.publishers[topic_name] = ros_pub

    config.publishers[topic_name].publish(msg)

    return msg
