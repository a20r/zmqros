
import urllib
import urllib2
import json


class NameServer(object):

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.address = "http://{}:{}".format(host, port)

    def get_new_connections(self, name):
        req = urllib2.urlopen(self.address + "/connections/new/" + name)
        resp = req.read()
        return json.loads(resp)

    def get_alive(self):
        req = urllib2.urlopen(self.address + "/alive")
        resp = req.read()
        resp_dict = json.loads(resp)
        return resp_dict

    def get_id(self, name):
        req = urllib2.urlopen(self.address + "/id/" + name)
        resp = req.read()
        resp_dict = json.loads(resp)
        robot_id = resp_dict["id"]

        if robot_id is None:
            raise LookupError(
                "Robot name, {}, does not exist in the database".format(name)
            )

        return robot_id

    def get_names(self):
        req = urllib2.urlopen(self.address + "/all")
        resp = req.read()
        bot_names = json.loads(resp)

        return bot_names

    def create_swarm(self, names):
        try:
            post_form = {
                "names": names
            }

            post_str = urllib.urlencode(post_form)
            req = urllib2.Request(self.address + "/swarm/create", post_str)
            resp = urllib2.urlopen(req)
            ret_dict = json.loads(resp.read())

            if not ret_dict["error"] == 0:
                raise Exception(ret_dict["message"])

            return ret_dict["host"], ret_dict["ports"]
        except urllib2.URLError:
            raise Exception("Unable to connect to name server")

    def free_swarm(self, name_port_dicts):
        try:
            post_form = {"swarm": name_port_dicts}
            post_str = urllib.urlencode(post_form)
            req = urllib2.Request(self.address + "/swarm/free", post_str)
            resp = urllib2.urlopen(req)
            ret_dict = json.loads(resp.read())

            if not ret_dict["error"] == 0:
                raise Exception(ret_dict["message"])

            return 0
        except urllib2.URLError:
            return 1
