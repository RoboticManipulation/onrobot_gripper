#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.msg import OnRobotRGInput

busy = 0
stop = 0
counter = 0
forces = [0]*6

def mainloop():

    rospy.init_node('TestNode', log_level=rospy.DEBUG)
    pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    command = OnRobotRGOutput()

    rospy.Subscriber("OnRobotRGInput", OnRobotRGInput, getSignals)
    rospy.sleep(0.1)

    global stop
    global busy
    global forces
    print("Start loop")
    while not rospy.is_shutdown():
        print("Busy? ",busy)
        if stop:
            stop = 0
            #stopping the movement
            print("\nSTOP")
            command.rCTR = 0
            rospy.sleep(0.1)

        elif not busy:
            command = generateCommands(command)

        pub.publish(command)
        rospy.sleep(0.001)

def getSignals(status):

    global busy
    global forces

    busy = status.Busy
    forces = [0]*6
    forces[0]=status.Fx_l
    forces[1]=status.Fy_l
    forces[2]=status.Fz_l
    forces[3]=status.Fx_r
    forces[4]=status.Fy_r
    forces[5]=status.Fz_r

def generateCommands(command):

    max_force = 400
    max_width = 1000
    global stop
    global busy
    global forces
    global counter

    if counter == 0:
        #activate gripper (open wit max force and set all foces to 0)
        print("\nactivating gripper")
        command.rGFR = max_force
        command.rGWD = max_width
        command.rCTR = 1
        command.outZero = 1
        counter += 1
        #rospy.sleep(0.1)

    elif counter == 1:
        #set a new force value
        print("\nSetting force to 30N")
        command.rGFR = 300
        counter += 1
        #rospy.sleep(0.1)

    elif counter == 2:
        #fully closing gripper
        print("\nfully closing the gripper")
        command.rGWD = 0
        command.rCTR = 1
        counter += 1
        
        #rospy.sleep(0.1)


    elif counter == 3:
        rospy.sleep(1)
        #stopping the movement
        while ((abs(forces[2])<100) and (abs(forces[5]<100))): # Fz_l > 10N
            #print("force = ",forces[2])
            rospy.sleep(0.001)
        stop = 1
        #continuing the movement
        print("\nstop the movement")
        command.rCTR = 1
        counter += 1

    elif counter == 4:
        #printing some sensor readings
        output = '\nHere are all force values:'
        output += ' Fx_l = ' + str(forces[0])
        output += ' Fy_l = ' + str(forces[1]) 
        output += ' Fz_l = ' + str(forces[2]) 
        output += ' Fx_r = ' + str(forces[3]) 
        output += ' Fy_r = ' + str(forces[4]) 
        output += ' Fz_r = ' + str(forces[5])
        print(output)
        counter += 1

    elif counter == 5: 
        print("\nshutting down")
        rospy.signal_shutdown("\n")

    return command


if __name__ == '__main__':
    gtype = rospy.get_param('/onrobot/gripper', 'rg2')
    mainloop()
