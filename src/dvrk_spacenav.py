#!/usr/bin/env python
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import Joy
import rospy
import dvrk
import PyKDL

psm = None
arm_wrench_spatial_pub = None


def spacenav_joy_callback(msg):
    trans = PyKDL.Vector(- msg.axes[1], msg.axes[0], msg.axes[2]) * 0.1
    rot = PyKDL.Rotation.RPY(msg.axes[0], msg.axes[1], msg.axes[2])
    delta_frame = PyKDL.Frame(rot, trans)
    psm.dmove(delta_frame, blocking=False)

    if msg.buttons[1]:
        rospy.loginfo('Opening jaw')
        psm.open_jaw(blocking=False)
    elif msg.buttons[0]:
        rospy.loginfo('Closing jaw')
        psm.close_jaw(blocking=False)

if __name__ == '__main__':
    rospy.init_node('dvrk_spacenav')
    psm_name = rospy.get_param('spacenav_arm_name', default='PSM1')
    psm = dvrk.psm(psm_name)
    spacenav_joy_sub = rospy.Subscriber('/spacenav/joy', Joy, spacenav_joy_callback)
    rospy.spin()