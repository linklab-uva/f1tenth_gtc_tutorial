#!/usr/bin/env python
'''
2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

banner = """
  Moving around
-----------------
        w
   a    s    d
-----------------
"""

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 0.5
turn = 0.25

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
  rospy.init_node('keyop')

  x = 0
  th = 0
  status = 0

  try:
    while True:
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = Twist();
       msg.linear.x = x*speed
       msg.angular.z = th*turn
       pub.publish(msg)

  except:
    print 'error'

  finally:
    msg = Twist();
    msg.linear.x = x*speed
    msg.angular.z = th*turn
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
