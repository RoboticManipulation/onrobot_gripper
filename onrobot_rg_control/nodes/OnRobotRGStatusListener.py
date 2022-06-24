#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGInput


def printStatus(status):
    """Prints the status string generated by the statusInterpreter function."""

    rospy.loginfo(statusInterpreter(status))  

def OnRobotRGStatusListener():
    """Initializes the node and subscribe to the OnRobotRGInput topic."""

    rospy.init_node('OnRobotRGStatusListener', log_level=rospy.DEBUG)
    rospy.Subscriber("OnRobotRGInput", OnRobotRGInput, printStatus)
    rospy.spin()


def statusInterpreter(status):
    """Generates a string according to the current value
       of the status variables.
    """

    output = '\n-----\nOnRobot RG status interpreter\n-----\n'

    # gFOF
    #output += 'gFOF = ' + str(status.gFOF) + ': '
    #output += 'Current fingertip offset: ' + str(status.gFOF / 10.0) + ' mm\n'

    # gGWD
    #output += 'gGWD = ' + str(status.gGWD) + ': '
    #output += 'Current width between the gripper fingers (w/o offset): ' + \
    #          str(status.gGWD / 10.0) + ' mm\n'

    # gSTA
    #output += 'gSTA = ' + str(status.gSTA) + ': '
    #gSTA16bit = format(status.gSTA, '016b')
    #output += '(gSTA (16 bit) = ' + gSTA16bit + '), Currtent states: '
    #if int(gSTA16bit[-1]):
    #    output += ' A motion is ongoing so new commands are not accepted.'
    #if int(gSTA16bit[-2]):
    #    output += ' An internal- or external grip is detected.'
    #if int(gSTA16bit[-3]):
    #    output += ' Safety switch 1 is pushed.'
    #if int(gSTA16bit[-4]):
    #    output += ' Safety circuit 1 is activated so the gripper cannot move.'
    #if int(gSTA16bit[-5]):
    #    output += ' Safety switch 2 is pushed.'
    #if int(gSTA16bit[-6]):
    #    output += ' Safety circuit 2 is activated so the gripper cannot move.'
    #if int(gSTA16bit[-7]):
    #    output += ' Any of the safety switch is pushed.'

    # gWDF
    #output += '\ngWDF = ' + str(status.gWDF) + ': '
    #output += 'Current width between the gripper fingers (w offset): ' + \
    #          str(status.gWDF / 10.0) + ' mm\n'

    #combined status signal
    output += '\ngSTA = ' + str(status.gSTA) +':Current combined status of the 2 fingers and 2 proximity sensors. If it is 0 everything is fine.\n\n'

    # here the new singals start
    output += 'StaFing_l = ' + str(status.StaFing_l) + ': Current Status of the left Finger, should be 0\n'
    output += 'StaFing_r = ' + str(status.StaFing_r) + ': Current Status of the right Finger, should be 0\n'
    output += 'StaProx_l = ' + str(status.StaProx_l) + ': Current Status of the left proximity sensor, should be 0\n'
    output += 'StaProx_r = ' + str(status.StaProx_r) + ': Current Status of the right proximity sensor, should be 0\n'
    output += 'Busy = ' + str(status.Busy) + ': Current gripper status. 0 = idle, accepting commands / 1 = busy, not accepting new commands\n'
    output += 'GripDet = ' + str(status.GripDet) + ': A 1 indicates that there currently is an external or internal grip being detected\n'
    output += 'inZero = ' + str(status.inZero) + ': If the value is 1, all force and torque values will be set to 0\n\n'

    output += 'ProxOff_l = ' + str(status.ProxOff_l) + ': Set proximity offset: ' + str(status.ProxOff_l / 10.0) + "mm \n"
    output += 'ProxOff_r = ' + str(status.ProxOff_r) + ': Set proximity offset: ' + str(status.ProxOff_r / 10.0) + "mm \n\n"

    output += 'Fx_l = ' + str(status.Fx_l) + ': Current force along the x axis on the left finger: ' + str(status.Fx_l / 10.0) + "N \n"
    output += 'Fy_l = ' + str(status.Fy_l) + ': Current force along the y axis on the left finger: ' + str(status.Fy_l / 10.0) + "N \n"
    output += 'Fz_l = ' + str(status.Fz_l) + ': Current force along the z axis on the left finger: ' + str(status.Fz_l / 10.0) + "N \n"
    output += 'Fx_r = ' + str(status.Fx_r) + ': Current force along the x axis on the right finger: ' + str(status.Fx_r / 10.0) + "N \n"
    output += 'Fy_r = ' + str(status.Fy_r) + ': Current force along the y axis on the right finger: ' + str(status.Fy_r / 10.0) + "N \n"
    output += 'Fz_r = ' + str(status.Fz_r) + ': Current force along the z axis on the right finger: ' + str(status.Fz_r / 10.0) + "N \n\n"

    output += 'Tx_l = ' + str(status.Tx_l) + ': Current torque about the x axis on the left finger: ' + str(status.Tx_l / 100.0) + "Nm \n"
    output += 'Ty_l = ' + str(status.Ty_l) + ': Current torque about the y axis on the left finger: ' + str(status.Ty_l / 100.0) + "Nm \n"
    output += 'Tz_l = ' + str(status.Tz_l) + ': Current torque about the z axis on the left finger: ' + str(status.Tz_l / 100.0) + "Nm \n"
    output += 'Tx_r = ' + str(status.Tx_r) + ': Current torque about the x axis on the right finger: ' + str(status.Tx_r / 100.0) + "Nm \n"
    output += 'Ty_r = ' + str(status.Ty_r) + ': Current torque about the y axis on the right finger: ' + str(status.Ty_r / 100.0) + "Nm\n"
    output += 'Tz_r = ' + str(status.Tz_r) + ': Current torque about the z axis on the right finger: ' + str(status.Tz_r / 100.0) + "Nm \n\n"

    output += 'Prox_l = ' + str(status.Prox_l) + ': Current proximity measurment of the left Finger: ' + str(status.Prox_l / 10.0) + "mm \n"
    output += 'Prox_r = ' + str(status.Prox_r) + ': Current proximity measurment of the right Finger: ' + str(status.Prox_r / 10.0) + "mm \n\n"
    output += 'GripWidth = ' + str(status.GripWidth) + ': Current width between the gripper fingers without any offset: ' + str(status.GripWidth / 10.0) + "mm \n\n"


    return output


if __name__ == '__main__':
    OnRobotRGStatusListener()
