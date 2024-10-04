import sys
import time
import rospy
import std_msgs.msg as ros_std_msgs

import ros_helper


def read_sensor_over_uart() -> str:
    ''' this function will simulate reading a value over uart from the robot MCU board '''
    return f"timestamp-{round(time.time())}"


if __name__ == '__main__':
    if not ros_helper.init_node('sensor_tx_node', ros_helper.RosMachine.RCS_MACHINE):
        sys.exit(1)

    sensor_tx_publisher: rospy.Publisher = rospy.Publisher('/robot/sensor', ros_std_msgs.String, queue_size=10)
    # /robot/sensor: sensor_tx_node publisher topic
    # ros_std_msgs.String: message type
    # The queue size used for asynchronously publishing messages from different threads

    while not rospy.is_shutdown():
        try:
            sensor_value = read_sensor_over_uart()
            sensor_tx_publisher.publish(sensor_value)
            time.sleep(2)

        except rospy.ROSInterruptException:
            break
