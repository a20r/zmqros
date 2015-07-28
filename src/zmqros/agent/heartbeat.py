
import urllib
import urllib2
import json
import time
import threading


class Heartbeat(object):

    def __init__(self, name, ns_host, ns_port):
        self.beat_url = self.get_beat_url(ns_host, ns_port)
        self.name = name
        self.delay = 1  # second
        self.beating = False

    def get_beat_url(self, ns_host, ns_port):
        ns_route = "http://{}:{}".format(ns_host, ns_port)
        return ns_route + "/alive"

    def beat(self):
        try:
            post_form = {
                "name": self.name
            }

            post_str = urllib.urlencode(post_form)
            req = urllib2.Request(self.beat_url, post_str)
            resp = urllib2.urlopen(req)
            return json.loads(resp.read())
        except urllib2.URLError:
            pass

    def live(self):
        while self.beating:
            self.beat()
            time.sleep(self.delay)

    def start(self):
        self.beating = True
        heart = threading.Thread(target=self.live)
        heart.daemon = True
        heart.start()

    def kill(self):
        self.beating = False
