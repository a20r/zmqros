
import util
import datetime


def get_current_time():
    return str(datetime.datetime.now())


@util.route("topic")
def log_topic(form):
    print "topic", " :: ",
    print get_current_time(), " :: ",
    print form["msg_type"], "::",
    print form["topic_name"]
