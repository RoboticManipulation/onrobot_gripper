#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGOutput


def genCommand(char, command):
    """Updates the command according to the character entered by the user."""

    if gtype == 'rg2':
        max_force = 400
        max_width = 1000
    elif gtype == 'rg6':
        max_force = 1200
        max_width = 1600
    else:
        rospy.signal_shutdown(
            rospy.get_name() +
            ": Select the gripper type from rg2 or rg6.")

    if char == 'a':
        command.rGFR = 200
        command.rGWD = max_width
        command.rCTR = 1
        command.outZero = 1
    if char == 'c':
        command.rGWD = 0
        command.rCTR = 1
    elif char == 'o':
        command.rGWD = max_width
        command.rCTR = 1
    elif char == 'i':
        command.rGFR += 25
        command.rGFR = min(max_force, command.rGFR)
    elif char == 'd':
        command.rGFR -= 25
        command.rGFR = max(0, command.rGFR)
    elif char == 's':
        command.rCTR = 0
    elif char == 'z':
        command.outZero = 1
    elif char == 'uz':
        command.outZero = 0
    elif char != '': 
        if char[0] == 'f':
            try:
                command.rGFR = max(0, min(max_force, int(char[1:])))
                command.rCTR = 1
            except ValueError:
                pass
        else:
            # If the command entered is a int, assign this value to rGWD
            try:
                command.rGWD = min(max_width, int(char))
                command.rCTR = 1
            except ValueError:
                pass

    return command


def askForCommand(command):
    """Asks the user for a command to send to the gripper."""

    currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
    currentCommand += ' rGFR = ' + str(command.rGFR)
    currentCommand += ', rGWD = ' + str(command.rGWD)
    currentCommand += ', rCTR = ' + str(command.rCTR)
    currentCommand += ', outZero = ' + str(command.outZero)

    rospy.loginfo(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 's: STOP current motion\n'
    strAskForCommand += '(0 - max width): Go to that position\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    strAskForCommand += 'z: sets the outZero bit to 1, so all force and torque values are set to 0\n'
    strAskForCommand += 'uz: sets the outZero bit to 0, to undo the force and torque bias\n'
    strAskForCommand += 'f0 to f400: Set the force value between 0N and 40N (very low values might result in the gripper not moving)\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """Main loop which requests new commands and
       publish them on the OnRobotRGOutput topic.
    """

    rospy.init_node('OnRobotRGSimpleController', log_level=rospy.DEBUG)
    pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    command = OnRobotRGOutput()

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    gtype = rospy.get_param('/onrobot/gripper', 'rg6')
    publisher()
