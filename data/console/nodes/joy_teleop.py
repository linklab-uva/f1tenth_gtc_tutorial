#!/usr/bin/env python
'''
2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

ANGLE_AXIS = 2
SPEED_AXIS = 1

def teleop(data):
    command           = Twist()
    command.angular.z = 1.0 * data.axes[ANGLE_AXIS]
    command.linear.x  = data.axes[SPEED_AXIS]
    cmd_pub.publish(command)

if __name__ == '__main__':
    try:
        rospy.init_node('joy_teleop')
        rospy.Subscriber('joy', Joy, teleop)
        rospy.spin()
    except rospy.ROSException:
        pass
