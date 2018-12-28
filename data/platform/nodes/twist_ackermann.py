#!/usr/bin/env python
'''
2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

angle_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size = 1)
speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size = 1)

VESC_SPEED_REF = 10000.0
VESC_ANGLE_MIN = 0.0
VESC_ANGLE_MAX = 1.0

def twist_ackermann(data):
    angle_ref      = Float64()
    speed_ref      = Float64()
    angle_ref.data = float(data.angular.z/2.0 + 0.5)
    speed_ref.data = float(data.linear.x  * VESC_SPEED_REF)
    angle_pub.publish(angle_ref)
    speed_pub.publish(speed_ref)

if __name__ == '__main__':
    try:
        rospy.init_node('twist_ackermann')
        rospy.Subscriber('cmd_vel', Twist, twist_ackermann)
        rospy.spin()
    except rospy.ROSException:
    	pass
