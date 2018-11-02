#!/usr/bin/env python

import roslib
import rospy
from robotiq_3f_gripper_control.msg import Robotiq3FGripper_robot_output


def publisher():
    rospy.init_node('CloseGripper')
    pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripper_robot_output, queue_size=1)

    command = Robotiq3FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150
    command.rPRA = 0

    rospy.sleep(1)
    pub.publish(command)

if __name__ == '__main__':
    publisher()
