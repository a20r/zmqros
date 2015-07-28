
import config


def run(host, port):
    config.app.run(host, port, debug=True, use_reloader=False)
