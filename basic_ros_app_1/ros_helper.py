import os
import rospy
from enum import Enum


class RosMachine(Enum):
    MCP_MACHINE = '192.168.122.1'
    RCS_MACHINE = '192.168.122.17'


def init_node(node_name: str, target_machine: RosMachine) -> bool:
    # setup environment variables
    os.environ['ROS_MASTER_URI'] = f"http://{RosMachine.MCP_MACHINE.value}:11311/"  # this URI should point to ROS master
    os.environ['ROS_IP'] = target_machine.value  # this IP should point to the machine running the node

    print(f"Initializing ROS Node {node_name}...")
    try:
        rospy.init_node(node_name, anonymous=True)
        print(f"Initializing ROS Node {node_name}...OK")
        return True

    except Exception as err:
        print(f"Initializing ROS Node {node_name}...ERR")
        print(err)
        return False
