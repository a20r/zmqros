
import config
import json
import time
from flask import request, jsonify
import util


@config.app.route("/swarm/create", methods=["POST"])
@util.crossdomain(origin="*")
def create_swarm():
    """
    Dynamically allocates ports for ZeroMQ channels

    Input: {
        "names": [<String: List of robot names in the swarm>, ...]
    }

    Output: {
        "error": <Int: Error code>,
        "message": <String: Associated error message>,
        "host": <String: hostname of requesting computer>,
        "swarm": [
            {
                "name": <String: Robot name>,
                "port": <Int: Port for ZMQ>
            }
        ]
    }

    Comments: This allocation needs a bit more optimization for large
    scale use

    """

    hostname = request.remote_addr
    new_ports = list()
    names = util.convert_string_to_list(request.form["names"])
    size = len(names)

    if not hostname in config.available_ports.keys():
        config.available_ports[hostname] = range(
            config.min_port, config.max_port
        )

    free_ports = config.available_ports[hostname]

    if size > len(free_ports):
        error_message = "Not enough free ports available ({} > {})"\
            .format(size, len(free_ports))
        return jsonify(error=1, message=error_message, swarm=list())
    else:
        new_ports = free_ports[:size]
        del free_ports[:size]

        for name, port in zip(names, new_ports):
            if not name in config.robot_connections.keys():
                config.robot_connections[name] = set()
                config.new_connections[name] = set()

            config.robot_connections[name]\
                .add(config.Connection(hostname, port))

            config.new_connections[name]\
                .add(config.Connection(hostname, port))

        return jsonify(
            error=0, message="No error", host=hostname, ports=new_ports
        )


@config.app.route("/swarm/free", methods=["POST"])
@util.crossdomain(origin="*")
def free_swarm():
    """
    Route that frees ports used by an application to communicate with
    a swarm through ZeroMQ

    Input: {
        "swarm": [
            {
                "port": <Int>,
                "name": <String>
            }
        ]
    }

    Output: {
        "error": <Int>,
        "message": <String>
    }

    """
    try:
        hostname = request.remote_addr
        swarm_strs = request.form["swarm"]
        swarm = json.loads(swarm_strs.replace("'", "\""))
        ports = map(lambda bot: bot["port"], swarm)
        free_ports = config.available_ports[hostname]
        free_ports_set = set(free_ports)
        free_ports_set.update(ports)
        config.available_ports[hostname] = list(free_ports_set)

        for bot in swarm:
            try:
                config.robot_connections[bot["name"]]\
                    .remove(config.Connection(hostname, bot["port"]))

                config.new_connections[bot["name"]]\
                    .remove(config.Connection(hostname, bot["port"]))

            except KeyError:
                pass

        return jsonify(error=0, message="No error")
    except Exception as e:
        return jsonify(error=1, message=str(e))


@config.app.route("/connections/new/<name>", methods=["GET"])
@util.crossdomain(origin="*")
def get_new_connections(name):
    """
    Gets the new connections for a robot

    Input: <name> --> String

    Output: [
        {
            "host": <String>,
            "port": <Int>
        }, ...
    ]

    """
    try:
        conns = list(config.new_connections[name])
        config.new_connections[name] = set()
        dict_conns = map(
            lambda conn: {"host": conn.host, "port": conn.port},
            conns
        )
        return json.dumps(dict_conns)
    except KeyError:
        return json.dumps(list())


@config.app.route("/alive", methods=["GET"])
@util.crossdomain(origin="*")
def get_alive():
    """
    Route that returns a address information of all the robots who have
    checked in with the server in the last 5 seconds. This shows the
    robots that are "alive".

    Input: None

    Output: [
        {
            "name": <String>
        }, ...
    ]

    """

    name_list = list()
    current_time = time.time()
    for name, check_in_time in config.live_robots.iteritems():
        time_diff = current_time - check_in_time
        if time_diff <= config.heartbeat_delay:
            name_list.append(name)

    return json.dumps(name_list)


@config.app.route("/alive/<name>", methods=["GET"])
@util.crossdomain(origin="*")
def get_robot_alive(name):
    """
    Route that checks if a robot is alive

    Input: <name> --> String

    Output: Boolean

    """
    try:
        time_diff = time.time() - config.live_robots[name]
        return jsonify(alive=time_diff < config.heartbeat_delay)
    except KeyError:
        return jsonify(alive=False)


@config.app.route("/alive", methods=["POST"])
@util.crossdomain(origin="*")
def post_alive():
    """
    Route that allows for alive robots to post update the main server
    if they are alive.

    Input: {
        "name": <String>
    }

    Output: {
        "error": <Int>,
        "message": <String>
    }

    """

    name = request.form["name"]
    config.live_robots[name] = time.time()
    return jsonify(error=0, message="No error")


@config.app.route("/all", methods=["GET"])
@util.crossdomain(origin="*")
def get_all_names():
    """
    Gets the names of all the robots

    Input: None

    Output: [<String: Name>, ...]

    """
    return json.dumps(config.live_robots.keys())
