#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGInput

def convertTwoComplement(val):
    """"Convert the uint16 signal to a signed interger"""
    bits=16
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val


class onrobotbaseRG:
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot RG gripper.
    """

    def __init__(self, gtype):
        # Initiate output message as an empty list
        self.gtype = gtype
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1000
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the gripper type from rg2 or rg6.")

        command.rGFR = max(0, command.rGFR)
        command.rGFR = min(max_force, command.rGFR)
        command.rGWD = max(0, command.rGWD)
        command.rGWD = min(max_width, command.rGWD)

        # Verify that the selected mode number is available
        if command.rCTR not in [0, 1]:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the mode number from" +
                "1 (grip), 0 (stop).")

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Updates the command which will be sent
           during the next sendCommand() call.
        """

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        self.message.append(command.outZero)
        self.message.append(command.rGFR)
        self.message.append(command.rGWD)
        self.message.append(command.rCTR)
        #self.message.append(command.outProxOff_l)
        #self.message.append(command.outProxOff_r)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def setProximityOffset(self, ProxOffsets):

        self.client.setProximityOffset(ProxOffsets)

    def getStatus(self):
        """Requests the status from the gripper and
           return it in the OnRobotRGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotRGInput()

        # Assign the values to their respective variables

        #original signlas left here to not break anything for now, adjusted the corosponding register number
        message.gFOF = 0              #There is no similar value for the rg2_ft
        message.gGWD = status[25]       
        message.gWDF = status[25]       #There is only one register for the actual gripper width, so gWDF = gGWD

        #there are now 4 status signals, which we will combine 

        if status[2] == 0 and status[11] == 0 and status[19] == 0 and status[22] == 0:
            message.gSTA = 0
        else:
            message.gSTA = 1

        #new signals
        message.StaFing_l = status[2]
        message.StaFing_r = status[11]
        message.StaProx_l = status[19]
        message.StaProx_r = status[22]

        message.ProxOff_l = status[0]
        message.ProxOff_r = status[1]

        message.Fx_l = convertTwoComplement(status[4])
        message.Fy_l = convertTwoComplement(status[5])
        message.Fz_l = convertTwoComplement(status[6])
        message.Tx_l = convertTwoComplement(status[7])
        message.Ty_l = convertTwoComplement(status[8])
        message.Tz_l = convertTwoComplement(status[9])
        message.Fx_r = convertTwoComplement(status[13])
        message.Fy_r = convertTwoComplement(status[14])
        message.Fz_r = convertTwoComplement(status[15])
        message.Tx_r = convertTwoComplement(status[16])
        message.Ty_r = convertTwoComplement(status[17])
        message.Tz_r = convertTwoComplement(status[18])
        message.Prox_l = convertTwoComplement(status[20])
        message.Prox_r = convertTwoComplement(status[23])
        message.GripWidth = convertTwoComplement(status[25])
        message.Busy = status[26]
        message.GripDet = status[27]
        message.inZero = status[28]

       
        return message
