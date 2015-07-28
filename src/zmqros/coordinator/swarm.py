
import master
import ns
import atexit


class Swarm(object):

    def __init__(self, host):
        self.host = host
        self.masters = dict()
        self.name_port_dicts = list()

    def add_bot(self, name, port):
        new_master = master.Master(self.host, port)
        self.masters[name] = new_master
        self.name_port_dicts.append({"name": name, "port": port})

    def send_message(self, bot_name, msg_type, topic_name, msg):
        self.masters[bot_name].send_message(msg_type, topic_name, msg)
        return self

    def get_names(self):
        return self.masters.keys()

    def get_bots(self):
        return self.masters.values()

    def get_dicts(self):
        return self.name_port_dicts

    def __getitem__(self, name):
        return self.masters[name]

    def __len__(self):
        return len(self.get_names())

    def __str__(self):
        return str(self.get_names())


class open_swarm(object):

    def __init__(self, ns_host, ns_port, names):
        self.ns_host = ns_host
        self.ns_port = ns_port
        self.names = names
        self.nserver = ns.NameServer(ns_host, ns_port)
        self.swarm = None

    def __enter__(self):
        host, ports = self.nserver.create_swarm(self.names)
        swarm = Swarm(host)

        for name, port in zip(self.names, ports):
            swarm.add_bot(name, port)

        self.swarm = swarm

        return swarm

    def __exit__(self, *args):
        self.nserver.free_swarm(self.swarm.get_dicts())


swarms_ref = dict()


def create_swarm_from_ns(ns_host, ns_port, names=None):
    global swarms_ref
    nserver = ns.NameServer(ns_host, ns_port)

    if names is None:
        names = nserver.get_names()

    host, ports = nserver.create_swarm(names)
    swarm = Swarm(host)

    if not (ns_host, ns_port) in swarms_ref.keys():
        swarms_ref[(ns_host, ns_port)] = list()

    swarms_ref[(ns_host, ns_port)].append(swarm)

    for name, port in zip(names, ports):
        swarm.add_bot(name, port)

    return swarm


@atexit.register
def free_swarms():
    for (ns_host, ns_port), swarms in swarms_ref.iteritems():
        nserver = ns.NameServer(ns_host, ns_port)
        for swarm in swarms:
            nserver.free_swarm(swarm.get_dicts())
