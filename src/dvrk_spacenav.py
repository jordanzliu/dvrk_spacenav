#!/usr/bin/env python
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import Joy
import rospy
import dvrk
import PyKDL
import Tkinter
import numpy as np

current_arm = None
arm_wrench_spatial_pub = None

def set_psm1_callback():
    global current_arm
    rospy.loginfo("Selected PSM1 for spacenav control")
    current_arm = dvrk.psm('PSM1')

def set_psm2_callback():
    global current_arm
    rospy.loginfo("Selected PSM2 for spacenav control")
    current_arm = dvrk.psm('PSM2')

def set_ecm_callback():
    global current_arm
    rospy.loginfo("Selected ECM for spacenav control")
    current_arm = dvrk.ecm('ECM')


def spacenav_joy_callback(msg):
    global current_arm
    if current_arm is not None:
        if isinstance(current_arm, dvrk.psm):
            # psm mappings
            trans = PyKDL.Vector(- msg.axes[1], msg.axes[0], msg.axes[2]) * 0.05
            rot = PyKDL.Rotation.RPY(msg.axes[0], msg.axes[1], msg.axes[2])
            delta_frame = PyKDL.Frame(rot, trans)

            current_arm.dmove(delta_frame, blocking=False)

            if msg.buttons[1]:
                rospy.loginfo('Opening jaw')
                current_arm.open_jaw(blocking=False)
            elif msg.buttons[0]:
                rospy.loginfo('Closing jaw')
                current_arm.close_jaw(blocking=False)
        else:
            # ecm mappings
            setup_joint_delta = - msg.axes[3] * 0.05
            pitch_joint_delta = - msg.axes[4] * 0.05
            roll_joint_delta = - msg.axes[5] * 0.05
            insertion_joint_delta = - msg.axes[2] * 0.05
            current_arm.dmove_joint(np.array([setup_joint_delta, pitch_joint_delta,
                                    insertion_joint_delta, roll_joint_delta]),
                                    blocking=False)



if __name__ == '__main__':
    rospy.init_node('dvrk_spacenav')
    set_psm1_callback()

    # initialize the UI
    root = Tkinter.Tk()
    root.geometry("200x200")
    frame = Tkinter.Frame(root)
    frame.pack()
    set_psm1_button = Tkinter.Button(frame, text="PSM1", command=set_psm1_callback)
    set_psm1_button.pack()
    set_psm2_button = Tkinter.Button(frame, text="PSM2", command=set_psm2_callback)
    set_psm2_button.pack()
    set_ecm_button = Tkinter.Button(frame, text="ECM", command=set_ecm_callback)
    set_ecm_button.pack()
    
    # start listening on ROS topic
    spacenav_joy_sub = rospy.Subscriber('/spacenav/joy', Joy, spacenav_joy_callback)
    root.mainloop()