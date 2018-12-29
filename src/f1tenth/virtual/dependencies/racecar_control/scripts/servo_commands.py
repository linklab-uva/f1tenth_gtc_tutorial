#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

flag_move = 0

def set_throttle_steer(data):
    global flag_move
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
    throttle = 5.0 * data.linear.x
    steer = 1.0 * data.angular.z
    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

if __name__ == '__main__':
    try:
        rospy.init_node('servo_commands', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, set_throttle_steer)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
