#!/usr/bin/env python

# Ride smoother
# Takes the desired heading angle from the FTG node
# and adjusts speed and steering rate before turns
#
# Author: Jaroslav Klapalek
# Copyright (C) 2018 Czech Technical University in Prague
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rospy
import math # less memory consuming than numpy

from simulator.msg import DriveValues, CarControlData
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64, Bool, Int16

# From `hw/teensy-drive/main.cpp`:
#define pwm_str_center_value  9375  //  15% duty cycle - V1 8611
#define pwm_str_lowerlimit    6040  //  10% duty cycle - V1 7000
#define pwm_str_upperlimit   11720  //  20% duty cycle - V1 10684

#define pwm_thr_center_value  8940  //  15% duty cycle - V1 8850
#define pwm_thr_lowerlimit    6567  //  10% duty cycle - V1 6458
#define pwm_thr_upperlimit   11312  //  20% duty cycle - V1 11241

pwm_pub = rospy.Publisher("drive_pwm", DriveValues, queue_size=10)
vesc_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=10)
ccd_pub = rospy.Publisher("ftg/data", CarControlData, queue_size=10)

# Publishers for car constants (instead of CarControlData.msg)
ftz_pub = rospy.Publisher("ftg/turn/zero", Int16, queue_size=1)
ftm_pub = rospy.Publisher("ftg/turn/min", Int16, queue_size=1)
fta_pub = rospy.Publisher("ftg/turn/max", Int16, queue_size=1)
ftv_pub = rospy.Publisher("ftg/turn/avoid", Int16, queue_size=1)

fsz_pub = rospy.Publisher("ftg/speed/zero", Int16, queue_size=1)
fsm_pub = rospy.Publisher("ftg/speed/min", Int16, queue_size=1)
fsa_pub = rospy.Publisher("ftg/speed/max", Int16, queue_size=1)
fsv_pub = rospy.Publisher("ftg/speed/avoid", Int16, queue_size=1)

fas_pub = rospy.Publisher("ftg/angle/switch", Float64, queue_size=1)
fah_pub = rospy.Publisher("ftg/angle/hyster", Float64, queue_size=1)
fvs_pub = rospy.Publisher("ftg/angle/switch_a", Float64, queue_size=1)
fvh_pub = rospy.Publisher("ftg/angle/hyster_a", Float64, queue_size=1)

fal_pub = rospy.Publisher("ftg/filter/alpha", Float64, queue_size=1)

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

# PWM duty for straight movement
VP_TURN_ZERO = 9375

# PWM duty for speed (lesser number ~ faster)
# Currently - AVOID is used during sharp turns, MIN is used during other turns, MAX elsewhere
# VP_SPEED_MIN = 6750
# VP_SPEED_MAX = 5750
# Fast extreme
#VP_SPEED_MIN = 7700
#VP_SPEED_MAX = 7100
#VP_SPEED_AVOID = 7700
# Fast extreme version 2
# VP_SPEED_MIN = 7650
# VP_SPEED_MAX = 7100
# VP_SPEED_AVOID = 7650
# Safe speed
# VP_SPEED_MIN = 7900
# VP_SPEED_MAX = 7900
# VP_SPEED_AVOID = 7900

#VP_SPEED_MIN = 7900
#VP_SPEED_MAX = 7700
#VP_SPEED_AVOID = 8000
VP_SPEED_MIN = 7900
VP_SPEED_MAX = 7850
VP_SPEED_AVOID = 8040

# Porsche constants
# VP_SPEED_MIN = 7800
# VP_SPEED_MAX = 7800 #7600
# VP_SPEED_AVOID = 7800 #7900

# PWM duty for turning
VP_TURN_MIN = -2100
VP_TURN_MAX = -3000 #nic nedela
VP_TURN_AVOID = -4000 #nic nedela

# Angle to switch between modes
ANGLE_SWITCH = math.pi * (20.0 / 180.0) # 20 degrees
ANGLE_HYSTER = math.pi * (5 / 180.0) # 5 degrees

ANGLE_SWITCH_A = math.pi * (41.0 / 180.0) # 40 degrees
ANGLE_HYSTER_A = math.pi * (5.0 / 180.0) # 5 degrees

mode = 0

VESC_speed = 4000

current_angle = VP_TURN_ZERO + VP_TURN_MAX
current_drive = VP_SPEED_MAX
FILTER_ALPHA = 0.00

str_lowerlimit = 6000
str_upperlimit = 11800

def angle_callback(angle):
    global mode
    global current_angle
    global current_drive

    VP_TURN_MAX = -3100
    VP_TURN_AVOID = -4400
    if angle.data > 0:
        VP_TURN_MAX = -2200
        VP_TURN_AVOID = -4000

    if math.isnan(angle.data):
         return

    DVmsg = DriveValues()
    VESCmsg = Float64()

    if (abs(angle.data) < (ANGLE_SWITCH - ANGLE_HYSTER)):
        mode = 0
    elif (abs(angle.data) > (ANGLE_SWITCH + ANGLE_HYSTER) and abs(angle.data) < (ANGLE_SWITCH_A - ANGLE_HYSTER_A)):
        mode = 1
    elif (abs(angle.data) > (ANGLE_SWITCH_A + ANGLE_HYSTER_A)):
        mode = 2

    # Go straight forward (high speed, low steering)
    if mode == 0:
        pwm_angle = VP_TURN_ZERO + VP_TURN_MIN * angle.data
        pwm_drive = VP_SPEED_MAX

    # Slow down before turning (low speed, high steering)
    elif mode == 1:
        pwm_angle = VP_TURN_ZERO + VP_TURN_MAX * angle.data
        pwm_drive = VP_SPEED_MIN

    # Slow down (a lot) before sharp turn (really low speed, really high steering)
    else:
        pwm_angle = VP_TURN_ZERO + VP_TURN_AVOID * angle.data
        pwm_drive = VP_SPEED_AVOID

    '''
    DVmsg.pwm_angle = current_angle * (1 - FILTER_ALPHA) + pwm_angle * (FILTER_ALPHA)
    DVmsg.pwm_drive = current_drive * (1 - FILTER_ALPHA) + pwm_drive * (FILTER_ALPHA)
    current_drive =  current_drive * (1 - FILTER_ALPHA) + pwm_drive * (FILTER_ALPHA)
    '''
    DVmsg.pwm_angle = pwm_angle
    DVmsg.pwm_drive = pwm_drive
    '''
    '''

    VESCmsg.data = VESC_speed

    pwm_angle = (pwm_angle - str_lowerlimit)/(str_upperlimit - str_lowerlimit)
    command.angular.z = (2.0 * pwm_angle) - 1.0

    if pwm_drive == VP_TURN_AVOID:
        command.linear.x = 0.2
    elif pwm_drive == VP_TURN_MIN:
        command.linear.x = 0.4
    elif pwm_drive == VP_SPEED_MAX:
        command.linear.x = 1.0

    cmd_pub.publish(command)

    pwm_pub.publish(DVmsg)
    vesc_pub.publish(VESCmsg)

def estop_callback(status):
    if status.data is False:

        # Post parameters
        ccdmsg = CarControlData()

        ccdmsg.vp_speed = VP_SPEED_MIN
        ccdmsg.vp_turn_zero = VP_TURN_ZERO
        ccdmsg.vp_turn_speed = VP_TURN_MAX

        ccdmsg.vesc_speed = VESC_speed

        ccd_pub.publish(ccdmsg)

        # Constants publishing
        msg_i8 = Int16()
        msg_f6 = Float64()

        msg_i8.data = VP_TURN_ZERO
        ftz_pub.publish(msg_i8)

        msg_i8.data = VP_TURN_MIN
        ftm_pub.publish(msg_i8)

        msg_i8.data = VP_TURN_MAX
        fta_pub.publish(msg_i8)

        msg_i8.data = VP_TURN_AVOID
        ftv_pub.publish(msg_i8)

        #msg_i8.data = VP_SPEED_ZERO
        #fsz_pub.publish(msg_i8)

        msg_i8.data = VP_SPEED_MIN
        fsm_pub.publish(msg_i8)

        msg_i8.data = VP_SPEED_MAX
        fsa_pub.publish(msg_i8)

        msg_i8.data = VP_SPEED_AVOID
        fsv_pub.publish(msg_i8)

        msg_f6.data = ANGLE_SWITCH
        fas_pub.publish(msg_f6)

        msg_f6.data = ANGLE_HYSTER
        fah_pub.publish(msg_f6)

        msg_f6.data = ANGLE_SWITCH_A
        fvs_pub.publish(msg_f6)

        msg_f6.data = ANGLE_HYSTER_A
        fvh_pub.publish(msg_f6)

        msg_f6.data = FILTER_ALPHA
        fal_pub.publish(msg_f6)


def listener():
    """Initialize ROS node and create subscriber callback."""
    rospy.init_node("ftg_ride", anonymous=True)
    rospy.Subscriber("/final_heading_angle", Float32, angle_callback)
    rospy.Subscriber("/eStop", Bool, estop_callback)

    rospy.spin()

if __name__ == "__main__":
    listener()
