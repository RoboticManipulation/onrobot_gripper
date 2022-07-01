#!/usr/bin/env python3

import rospy
import onrobot_rg_modbus_tcp.comModbusTcp
import onrobot_rg_control.baseOnRobotRG
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput


def mainLoop():
    # Gripper is a RG gripper with a Modbus/TCP connection
    gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype)
    gripper.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)

    # Connects to the ip address received as an argument
    gripper.client.connectToDevice(ip, port)
    rospy.init_node('OnRobotRGTcpNode', log_level=rospy.DEBUG)

    # The Gripper status is published on the topic named 'OnRobotRGInput'
    pub = rospy.Publisher('OnRobotRGInput', OnRobotRGInput, queue_size=1)

    # The Gripper command is received from the topic named 'OnRobotRGOutput'
    rospy.Subscriber('OnRobotRGOutput',
                     OnRobotRGOutput,
                     gripper.refreshCommand)

    # Set the Proximit Offsets
    ProxOffsets = []
    ProxOffsets.append(230)
    ProxOffsets.append(170)
    gripper.setProximityOffset(ProxOffsets)

    # We loop
    prev_msg = []
    while not rospy.is_shutdown():
        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)

        rospy.sleep(0.0005)
        # Send the most recent command
        if not int(format(status.gSTA, '016b')[-1]):  # not busy
            if not prev_msg == gripper.message:       # find new message
                rospy.loginfo(rospy.get_name()+": Sending message.")
                gripper.sendCommand()
        prev_msg = gripper.message
        rospy.sleep(0.0005)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        gtype = rospy.get_param('/onrobot/gripper', 'rg2')
        dummy = rospy.get_param('/onrobot/dummy', False)
        mainLoop()
    except rospy.ROSInterruptException:
        pass
