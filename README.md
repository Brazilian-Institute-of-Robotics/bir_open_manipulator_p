# **BIR OpenMANIPULATOR-PRO**

![ompBIR](https://user-images.githubusercontent.com/32513366/73663829-15deca00-467d-11ea-853e-bce2171e91f0.jpeg)

This repository consists of a ROS metapackage suited to the BIR (Brazilian Institute of Robotics) customized ROBOTIS OpenMANIPULATOR-PRO. Present content was forked and modified from original ROBOTIS repository - more information [here](https://github.com/ROBOTIS-GIT/open_manipulator_p).

## **Summary**
- [Robot Modifications](#robot-modifications)
- [Joint Limits](#joint-limits)
- [Robot arm Setup](#robot-arm-setup)
- [Dynamixel Wizard 2.0](#dynamixel-wizard-20)
- [MoveIt! Packages](#moveit-packages)
- [Requirements](#requirements)
- [Improvements](#improvements)
- [Minor issues](#minor-issues)

## **Robot Modifications**
OpenManipulator-PRO is a robot arm composed by 6 revolute joints, where each is a Dynamixel PRO:
![ompJOINTS](https://user-images.githubusercontent.com/32513366/73868912-123b7680-4828-11ea-9500-034c9449867c.png)

Source: [ROBOTIS](http://www.robotis.us/openmanipulator-pro/)

For BIR customized model, was added to OpenManipulator-PRO three modifications:
- **Metal base support**: Give a base support to manipulate the robotic arm.
- **Custom Gripper**: The original basic manipulator does not include an end-effector, so it was design one to meet our demand.
- **Camera**: Insert a visual sensor (camera) to perform different tasks based in detection.

### Custom Gripper
This tool is composed by
- 24-12 DC Conversor
- Dynamixel MX-28
- [FR07-G101GM Set](http://www.robotis.us/fr07-g101gm-set/)
- 3D printed support

The 24-12 DC Conversor was necessary because MX-28 works with 12V, different from any Dynamixel PRO that works with 24V. Because the connection is in serial, was necessary the convertion.

The Dynamixel MX-28 had your protocol updated from 1.0 to 2.0 to works correctly in Serial.

The support was created using [Onshape web software](https://cad.onshape.com/signin) with measurements based on technical [drawing from ROBOTIS manipulator](http://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/) and physical measures in FR07-G101GM Set using a paquimeter.

### Camera
This tool is composed by
- Teledyne Nano Series camera
- *Kowa* Lens LM6HC
- 3D printed support

The support was created using [Onshape web software](https://cad.onshape.com/signin) with measurements based on two technicals drawing:    
- [ROBOTIS manipulator](http://emanual.robotis.com/docs/en/dxl/pro/h42-20-s300-r/)
- [Teledyne camera](https://www.teledynedalsa.com/en/support/downloads-center/cad-drawings/).

### 3D Supports
To get the files developed to create 3D supports and meshes for URDF, search in *Onshape* for:
![3DS](https://user-images.githubusercontent.com/32513366/73663586-a5d04400-467c-11ea-9879-e8e6e5a26cf4.png)

## Joint Limits
For correct use and seeking to eliminate possible collision accidents after the insertion of customized items (gripper and camera), it was necessary to change all the joints limits in OpenManipulator-PRO. Those limits needs to be changed in all URDF (that start with **bir_**)  located in:
```
/open_manipulator_p_description
```
and in a ```.cpp``` file (related to physical limits for your robot):
```
/open_manipulator_p_libs/src/bir_open_manipulator_p.cpp
```
The table bellow present the actual limits with important warnings related to that joint:
| Joint Name | Min Limit | Max  Limit | Warning  |
| --------   | --------- | ---------- | ---- |
| Joint 1    | -90°      | +90°       | Keep in mind that a full rotation (-180° to +180°) can destroy robot power and communication physical cables |
| Joint 2    | -90°      | +90°       | Those limits prevent a self-collision or a table collision   |
| Joint 3    | -60°      | +120°      | It's necessary to apply -45° in ```.urdf``` pitch to match with ```.cpp``` file |
| Joint 4    | -175°     | +175°      | ---  |
| Joint 5    | -55°      | +55°       | Those limits are necessary to prevent a collision between the robot and camera |
| Joint 6    | -106°     | -21°       | Those limits are necessary to prevent destruction of 24-12V Conversor physical cables  |
| Gripper    | 0°        | 117°       | Maximum Limits to prevent a self-collision. 0° is **close** and 117° is **open** |

## Robot arm Setup
To use correctly our customized BIR OpenManipulator-PRO (arm and modifications) you are going to need:
- 1 DC Power Supply capable to get 24 Volts (recommended by ROBOTIS is 24V/15A, but for our applications 15A was not necessary)
- 1 Power Supply for Teledyne camera
- 1 Power cable to connect your Robot to Power Supply
- 1 Ethernet cable to connect your Teledyne camera to your computer
- 1 USB cable to connect your Robot to your computer
- 1 Printed support to rest your robot

Bellow you can see a Power Supply for your robot and camera:
![POWER](https://user-images.githubusercontent.com/32513366/73876287-803a6a80-4835-11ea-9fda-a38a40d841e3.jpeg)

Here you can see all connectors and the printed support at your right:
![conectors](https://user-images.githubusercontent.com/32513366/73876275-7ca6e380-4835-11ea-949f-ff1864d59d0e.jpeg)

Before you proceed with any power connection, is recommend to manually insert OpenManipulator-PRO in this position:
![ompPHOME](https://user-images.githubusercontent.com/32513366/73876283-7f093d80-4835-11ea-9a5d-6d9be0652609.jpeg)

PS: For your security, keep in mind the OpenManipulator workspace:
![ompWS](https://user-images.githubusercontent.com/32513366/73868913-12d40d00-4828-11ea-9add-7e510da677dc.png)

## **Dynamixel Wizard 2.0**
This software was developed by **ROBOTIS** and allows the user an easy control over the OpenManipulator joints. Your interface is shown bellow

![dw2](https://user-images.githubusercontent.com/32513366/73656756-8ed72500-466f-11ea-9788-5d7a6b8cb474.png)

First, I will briefly explain some important options in MENU:
- **Scan**: Search connected Dynamixels in your computer
- **Disconnect**: Disconnect a selected Dynamixel from your computer
- **Options**: Define the search parameters to find a specific Dynamixel as baud rate, USB port and so on.
- **Update**: Used to change an communication protocol for your Dynamixel.

After a successful **Scan** you can see at your superior-left a Dynamixels list with ID and model, at your superior-right a manual control to a Dynamixel that can be set with your mouse and in the middle, with a **disable torque** you can change several limits/info from your Dynamixel such as PID values, Max/Min velocities and positions.

For more useful information acess [here](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/), there you will have a link to download this software.

After installation, if you have a problem to connect your OpenManipulator-PRO through USB port try:

1 - Verify if you have connection through terminal command:
```sh
dmesg
```

2 - Allow your system to read your USB Port
```sh
sudo chmod 666 /dev/ttyUSB0
```

## **MoveIt! Packages**
To use any *MoveIt!* application for this custom model with a RBG Camera or Gripper with RGB Camera, get those repositories respectively in your workspace:
- [bir_open_manipulator_p_with_cam_moveit](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_cam_moveit.git) (master branch)
```sh
git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_cam_moveit.git
```
- [bir_open_manipulator_p_with_gripper_cam_moveit](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_gripper_cam_moveit.git) (master branch)
```sh
git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_gripper_cam_moveit.git
```
To evaluate any *MoveIt!* planner for original model:
- [bir_open_manipulator_p_moveit](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_moveit.git) (master branch)
```sh
git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_moveit.git
```

## **Requirements**

This repository was created using:
- ROS version - **Melodic**
- OS - **Ubuntu 18.04**

### ROS Packages
It is necessary to have **MoveIt! 1.0** installed

```sh    
sudo apt-get install ros-melodic-moveit
```
and some controllers packages:
```sh
sudo apt-get install ros-melodic-joy ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-controller-manager ros-melodic-joint-trajectory-controller ros-melodic-joint-state-controller
ros-melodic-position-controllers
```
It is strictly recommend to have those specifications, before you proceed.

### OpenManipulator-PRO Dependencies
First, to use this meta package create a new workspace:
```sh
mkdir -p ~workspaces/omp_ws/src
cd ~/workspaces/omp_ws/src
```
Then, clone this metapackage:
```sh
git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p.git

```
After that, install the dependencies that are necessary to use OpenManipulator-PRO and simulations with ROS/Gazebo:

- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (master branch)
    ```sh
    git clone -b master https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    ```

- [dynamixel-workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench) (master branch)
    ```sh
    git clone -b master https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
    ```

- [dynamixel-workbench-msgs](https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs) (master branch)
    ```sh
    git clone -b master https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
    ```

- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs) (master branch)
    ```sh
    git clone -b master https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
    ```

- [robotis_manipulator](https://github.com/ROBOTIS-GIT/robotis_manipulator) (master branch)
    ```sh
    git clone -b master https://github.com/ROBOTIS-GIT/robotis_manipulator.git
    ```
- [open_manipulator_pro_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_pro_simulations) (master branch)
  ``` sh
  git clone -b master https://github.com/ROBOTIS-GIT/open_manipulator_p_simulations.git
  ```

### Finishing
After all depedencies are present in your workspace, in a terminal:
```sh
cd ~/workspaces/omp_ws
catkin build
echo "source $HOME/workspaces/omp_ws/devel/setup.bash" > ~/.bashrc
source ~/.bashrc
rosdep install --from-paths src --ignore-src -r -y
```
Those commands install and update any missing requirement for your packages.

**PS**: You can use this software without any ROS connection. But, if you use this software, you can not use at the same time any ROS package to communicate with your robot.

## **Improvements**
This repository, as your MoveIt! packages have a lot of possibilities to improve more OpenManipulator-PRO development as:
- [ ] Rewrite all codes in MoveIt! packages based on [Python google code style](http://google.github.io/styleguide/pyguide.html). To Improve readability over current and future codes.
- [ ] Develop a **STOP button** capable to interrupt OpenManipulator-PRO MoveIt! execution in real world. Keep in my mind the problems that OpenManipulator-PRO has with MoveIt!
- [ ] Develop a python code file to work as a library to basic MoveIt! functions to control OpenManipulator-PRO through MoveIt!. This is necessary because most of the codes developed in any MoveIt! package here presents the same structure.

## **Minor issues**
In this repository there are several minor issues to fix (most related to visual effects and communication):
- [ ] Visualizing OpenManipulator-PRO with gripper and camera URDF in *RViz* shows a second camera floating over the space. It is necessary to eliminate it!
    - **Issue type**: Visual
    - **Additional info**: Could be a bug in meshe file.
- [ ] Gripper and Camera meshes don't present texture in *Gazebo*. Should be solved to present the same visual as in *Rviz*.
    - **Issue type**: Visual
    - **Additional info**: ---
- [ ] Change meshe file for gripper to be presented in *Gazebo* as the same in Real world.
    - **Issue type**: Visual and Mechanical
    - **Additional info**: There are two angular lag: Joint 6 with 3D base and 3D base with Gripper. This angular lag prevent the simulation to show the same results as in real world. For the first lag, it is recommended to change URDF and for the second one to fix in ```.dae``` file the lag.
- [ ] Discover and solve why USB communication with OpenManipulator-PRO some times just drop randomly
    - **Issue type**: Communication
    - **Additional info**: Some times, without any warning, the robot communication will drop. Could be the higher actual baud rate on every Dynamixels connected, USB port (2.0 vs 3.0) or a cable problem because this problem was verify in different computers.
- [ ] Discover and solve why teledyne RGB camera communication some times just drop randomly
    - **Issue type**: Communication
    - **Additional info**: Could be the RJ-45 cable fixation (it's connection is physically unstable), camera higher resolution or FPS.
- [ ] Discover why MoveIt! can not control the gripper in Real world tasks.
    - **Issue**: Communication/Technical
    - **Additional info**: Actually, to control the gripper in real world is necessary to use a ROS service. Besides that, MoveIt! is unable to control the gripper despite sending the commands. In simulation, the control works perfectly.
- [ ] Understand why MoveIt! sequential execution does not work properly in real world, despite working in *Gazebo*.
    - **Issue**: Technical
    - **Additional info**: The main problem here (the most important) is that MoveIt! in python does not respect the argument **wait=True** in ```.go``` function. This cause a communication problem, where different positions are executed at the same time. Nowadays, this problem was fixed based in a loop comparison over target state and joint actual state, but is not the ideal. 
- [ ] Discover and solve why there are tons of warnings about Control time using OpenManipulator-PRO (remove those warnings if possible)
    - **Issue**: Communication
    - **Additional info**: The argument related to this can be found in ```./open_manipulator_p_controller/src/bir_open_manipulator_p_controller.cpp``` as ```control_period_``` variable and in ```./open_manipulator_p_controller/launch/bir_open_manipulator_p_controller.launch``` as ```control_period``` parameter.
    
