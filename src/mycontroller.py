#!/usr/bin/env python
import roslib; roslib.load_manifest('navigate')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


def callback(data):
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    msg = Twist()
    rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f', data.ranges[88], data.ranges[89])
    msg.linear.x = 0.2
    left= data.ranges[134]
    right= data.ranges[44]
    if left<right:
        msg.angular.z = -0.5
    else:
        msg.angular.z = +0.5
    pub.publish(msg)


def listen():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()



if __name__ == '__main__':
    listen()
