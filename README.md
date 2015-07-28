ZeroMQ-ROS
========

ZeroMQ-ROS is a middleware that allows for the control of multiple ROS masters from
a single coordinator and makes it easy to create swarm applications using the standard
ROS framework. ZeroMQ-ROS uses the ZeroMQ message queue to communicate between the coordinator
and the agents.

## Architecture
#### Agents

Agents are members of the swarm that will be controlled by a coordinator. They are referenced by
a unique identifying name and listen for JSON serialized ROS messages on a message queue. These
serialized messages are then constructed into ROS messages and published to the associated topic.

#### Coordinators

Coordinators are the controllers of the swarm. They can send JSON serialized ROS messages to agents
that get published to the associated topic on the agent's ROS master. To enable bi-directional,
coordinators can also be agents and listen for messages.

#### Naming service

The naming service is used to associate a robot's unique name to a host and port of the ZeroMQ
message queue. This is vital for having dynamic swarm membership because instead of manually sharing
a configuration file, a persistent, centralized server runs that holds all of this information. Also,
the naming service holds a record of what agents are currently *alive* and able to be used.

## Install

**[Warning] You must already have ROS installed on all machines you would like
to have in your swarm**

#### Ubuntu
    $ git clone https://github.com/wallarelvo/zeromq-ros.git
    $ make
    
#### Other distributions
1. Install Ubuntu
2. Follow instructions for installation on Ubuntu
    
## Setting up the middleware
#### Environment Variables
In order to use ZeroMQ-ROS, you must set a few environmental variables. These can
be set by exporting environmental variables in the terminal every time you start a
new terminal, or you can be sane and add them to your `.bashrc` or `.bash_profile`.
You must set variables described below. These environment variables need to be set on
every computer running ZeroMQ-ROS.

- `ZMQROS_NS_HOST` -- Host of the naming service
- `ZMQROS_NS_PORT` -- Port of the naming service
- `ZMQROS_ROBOT_ID` -- A unique identifying name of the robot running ZeroMQ-ROS
- `ZMQROS_ROOT` -- The location of the root directory of the ZeroMQ-ROS installation

#### Database
Wherever you choose to run the name server, you must also be running a RethinkDB database.
This database stores the names and identification numbers of the agents you wish the name
server to supervise. For instance, if you work in a robotics lab, the database
will contain all of the names of the robots in your lab along with some identification number.
All of the names in the database do not need to be used at the same time, they are just there
for reference by the name server.
These names need to be known a priori for more efficient swarm creation and name allocation.
To do this run

    $ rethinkdb --bind all

This will run the RethinkDB instance. In order to populate the database, run Zui, the
ZeroMQ-ROS UI runnning,

    $ zmqros --ui
    
Then to populate the database visit, `localhost:9000/add`. The website will ask you for the
robot names and an identification number. This is useful so that you can link the robot
to IP addresses, Vicon identification and so on.

## Running

1. First run the name server and populate the database with robot information. This step does 
not need to occur every time you run the swarm, however, the name server needs to be running for
agents and coordinators to communicate. The agents and the coordinators need to have their
environment variables pointing to the address and port the name server will be running from.
    - `$ zmqros --ns`
2. On all of the agents in the swarm that are specified in the name server, run the agent code. 
This step also does not need to occur everytime the swarm is to be controlled, however to control an agent,
the agent program must be running.
    - `$ zmqros --agent`
3. Run the coordinator code from where ever the coordinator is located.
4. (Optional) Now you are able to control the swarm. To look at properties in the swarm, run Zui, the web app for the
middleware. Using Zui, you will be able to look at the members of the swarm that are currently alive and
send ROS messages to them through a GUI.
    - `$ zmqros --ui`
    

## Coordinator example

```python
import zmqros
from zmqros.coordinator import open_swarm
import random
import time
from geometry_msgs.msg import Twist

ns_host = zmqros.get_ns_host()
ns_port = zmqros.get_ns_port()
nserver = zmqros.coordinator.NameServer(ns_host, ns_port)
names = nserver.get_names()


def run():
    with open_swarm(ns_host, ns_port, names) as swarm:
        while True:
            for bot in swarm.get_bots():
                t = Twist()
                t.linear.x = random.random()
                t.linear.y = random.random()
                t.linear.z = random.random()
                t.angular.x = random.random()
                t.angular.y = random.random()
                t.angular.z = random.random()

                bot.send_message("geometry_msgs/Twist", "/cmd_vel", t)

            time.sleep(0.1)

if __name__ == "__main__":
    run()

```
