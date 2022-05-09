#!/usr/bin/env python3
import sys
import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


def publisher(input_command):
    rospy.init_node('OpenGripper')
    pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=1)
    command = Robotiq3FGripperRobotOutput()
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150
    if input_command == "o":
        command.rPRA = 0
    elif input_command == "c":
        command.rPRA = 255
    else:
        rospy.logwarn("use o or c to control the hand open or close")
        raise NotImplementedError
    rospy.sleep(1)
    pub.publish(command)


if __name__ == '__main__':
    publisher(input_command=sys.argv[1])
