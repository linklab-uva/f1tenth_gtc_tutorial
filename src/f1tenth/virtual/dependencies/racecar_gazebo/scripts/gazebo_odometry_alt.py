#!/usr/bin/env python
'''
2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

import rospy, tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped

odom_pub = rospy.Publisher('odom', Odometry, queue_size = 1)
tf_pub   = tf2_ros.TransformBroadcaster()

def odom_callback(data):
    odom = Odometry()
    odom.header.stamp    = rospy.Time.now()
    odom.header.frame_id = 'odom'      # 'map'
    odom.child_frame_id  = 'base_link' # 'odom'
    odom.pose            = data.pose
    odom.pose.pose.position.y = odom.pose.pose.position.y + 5.0
    odom.twist           = data.twist
    tf                   = TransformStamped(
                              header         = Header(
                              frame_id       = odom.header.frame_id,
                              stamp          = odom.header.stamp),
                              child_frame_id = odom.child_frame_id,
                              transform      = Transform(
                              translation    = odom.pose.pose.position,
                              rotation       = odom.pose.pose.orientation))
    odom_pub.publish(odom)
    tf_pub.sendTransform(tf)

if __name__ == '__main__':
    rospy.init_node("odometry_node")
    rospy.Subscriber('ground_truth', Odometry, odom_callback)
    rospy.spin()
