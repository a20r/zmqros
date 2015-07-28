

import urllib
import urllib2
import json


address = "http://132.250.85.150:8000"
current_port = 5556


def test_create_swarm():
    global current_port
    post_form = {
        "names": ["test"]
    }

    post_str = urllib.urlencode(post_form)
    req = urllib2.Request(address + "/swarm/create", post_str)
    resp = urllib2.urlopen(req)
    ret_dict = json.loads(resp.read())
    current_port = ret_dict["ports"][0]
    return ret_dict


def test_free_swarm():
    post_form = {
        "swarm": [
            {
                "port": current_port,
                "name": "test"
            }
        ]
    }

    post_str = urllib.urlencode(post_form)
    req = urllib2.Request(address + "/swarm/free", post_str)
    resp = urllib2.urlopen(req)
    return json.loads(resp.read())


if __name__ == "__main__":
    print test_create_swarm()
    # print test_free_swarm()
