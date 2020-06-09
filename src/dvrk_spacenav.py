#!/usr/bin/env python
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import Joy
import rospy
import dvrk
import PyKDL

psm = None
arm_wrench_spatial_pub = None

def spacenav_twist_callback(msg):
    linear = msg.linear
    angular = msg.angular
    trans = PyKDL.Vector(linear.x, linear.y, linear.z)
    rot = PyKDL.Rotation.RPY(angular.x, angular.y, angular.z)
    delta_frame = PyKDL.Frame(rot, trans)
    psm.dmove(delta_frame, blocking=False)

def spacenav_joy_callback(msg):
    # we only use the joy msg to get the button press for opening the jaw
    pass

if __name__ == '__main__':
    rospy.init_node('dvrk_spacenav')
    psm_name = rospy.get_param('spacenav_arm_name', default='PSM1')
    psm = dvrk.psm(psm_name)
    spacenav_twist_sub = rospy.Subscriber('/spacenav/twist', Twist, spacenav_twist_callback)
    spacenav_joy_sub = rospy.Subscriber('/spacejav/joy', Joy, spacenav_joy_callback)
    rospy.spin()