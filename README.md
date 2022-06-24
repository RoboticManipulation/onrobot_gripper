# OnRobot ROS1 Noetic Driver
ROS driver for OnRobot GR2-FT Gripper. The [original driver](https://github.com/Osaka-University-Harada-Laboratory/onrobot) was adapted to work only with the OnRobot RG2-FT gripper. All variables called rg2 are used for the rg2-ft.

## Features
- ROS Noetic (Python3)
- Controler for OnRobot RG2-FT via Modbus/TCP

## Installation
```
git clone git@github.com:RoboticManipulation/onrobot_gripper.git catkin_ws/src; cd catkin_ws
sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Usage

1. Connect the cable between Compute Box and Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Execute programs

### RG2-FT

##### Send motion commands
Start the interface:
``` 
roslaunch onrobot_rg_control bringup.launch gripper:=rg2
```

Send motion commands via terminal:
```
rosrun onrobot_rg_control OnRobotRGSimpleController.py
```

Execute test node to execute several predefined commands in a row:
```
rosrun onrobot_rg_control TestNode.py
```

## Authors / Contributors
Original driver: [Takuya Kiyokawa](https://takuya-ki.github.io/)

Adaptions for RG2-FT: [Benedikt Kreis](https://github.com/benediktkreis)

Adaptions for RG2-FT: [Patrick Reitz](https://github.com/PatrickReitz)


## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
