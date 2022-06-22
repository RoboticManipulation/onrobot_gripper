#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import time
import rospy
import threading
from pymodbus.client.sync import ModbusTcpClient


class communication:

    def __init__(self, dummy=False):
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()

    def connectToDevice(self, ip, port):
        """Connects to the client.
           The method takes the IP address and port number
           (as a string, e.g. '192.168.1.1' and '502') as arguments.
        """
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.client.connect()

    def disconnectFromDevice(self):
        """Closes connection."""
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client.close()

    def sendCommand(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
        """
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        # Send a command to the device (address 0 ~ 2)
        if message != []:
            with self.lock:
                self.client.write_registers(
                    address=2, values=message, unit=65)

    def getStatus(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
        """
        response1 = [0] * 2
        response2 = [0] * 26
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return response1 + response2

        # Get status from the device (address 5 and 6)
        with self.lock:
            response1 = self.client.read_holding_registers(
                address=5, count=2, unit=65).registers

        # Get status from the device (address 257 ~ 282)
        with self.lock:
            response2 = self.client.read_holding_registers(
                address=257, count=26, unit=65).registers

        # Output the result
        return response1 + response2
