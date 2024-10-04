import sys
import rospy
import time
import std_msgs.msg as ros_std_msgs

import ros_helper


def ros_subscriber_handler(msg: ros_std_msgs.String):
    print('robot_sensor:', msg.data)


if __name__ == '__main__':
    if not ros_helper.init_node('sensor_rx_node', ros_helper.RosMachine.MCP_MACHINE):
        sys.exit(1)

    sensor_rx_subscriber = rospy.Subscriber = rospy.Subscriber('/robot/sensor', ros_std_msgs.String, ros_subscriber_handler)
    # ros_std_msgs.String: message type
    # ros_subscriber_handler: this function get called whenever a message is received

    while not rospy.is_shutdown():
        try:
            time.sleep(1)

        except rospy.ROSInterruptException:
            break
