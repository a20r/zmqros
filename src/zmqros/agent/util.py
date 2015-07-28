
import config


class route(object):

    def __init__(self, route_name):
        self.route_name = route_name

    def __call__(self, f):
        try:
            config.callback_functions[self.route_name].append(f)
        except KeyError:
            config.callback_functions[self.route_name] = [f]
        return f


def create_addr(host, port):
    addr = "tcp://{}:{}".format(host, port)
    return addr
