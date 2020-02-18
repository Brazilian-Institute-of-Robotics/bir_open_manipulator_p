# **BIR OpenMANIPULATOR-PRO**

![ompBIR](https://user-images.githubusercontent.com/32513366/73663829-15deca00-467d-11ea-853e-bce2171e91f0.jpeg)

This repository consists of a ROS metapackage suited to the BIR (Brazilian Institute of Robotics) customized ROBOTIS OpenMANIPULATOR-PRO. Present content was forked and modified from original ROBOTIS repository - more information [here](https://github.com/ROBOTIS-GIT/open_manipulator_p).

## **Summary**
- [Robot Modifications](#robot-modifications)
- [Robot arm Setup](#robot-arm-setup)
- [Dynamixel Wizard 2.0](#dynamixel-wizard-20)
- [MoveIt! Packages](#moveit-packages)
- [Requirements](#requirements)

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
To use any *MoveIt!* application for this custom model with a RBG Camera or Gripper with RGB Camera, get those repositories respectively:
- [bir_open_manipulator_p_with_cam_moveit](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_cam_moveit.git) (master branch)
```sh
git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_cam_moveit.git
```
- [bir_open_manipulator_p_with_gripper_cam_moveit](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_gripper_cam_moveit.git) (master branch)
```sh
git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p_with_gripper_cam_moveit.git
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

### Workspace dependencies
To use correctly any package developed here, get those dependencies in your workspace:
- [apriltag](https://github.com/AprilRobotics/apriltag) (master branch)
    ```sh
    git clone -b master https://github.com/AprilRobotics/apriltag.git
    ```
- [apriltag_ros](https://github.com/AprilRobotics/apriltag) (master branch)
    ```sh
    git clone -b master https://github.com/AprilRobotics/apriltag_ros.git
    ```
- [def_cam_teledyne_nano](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano) (feature_flip branch)
    ```sh
    git clone -b feature_flip https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano.git
    ```

- [warehouse_ros_mongo](https://github.com/ros-planning/warehouse_ros_mongo.git) (melodic-devel branch)
    ```sh
    git clone -b melodic-devel https://github.com/ros-planning/warehouse_ros_mongo.git
    ```
- [qt_ros](https://github.com/stonier/qt_ros) (indigo branch)
    ```sh
    git clone -b indigo https://github.com/stonier/qt_ros.git
    ```
To use **warehouse-ros-mongo**, install in your **src** folder mongo dependency, necessary to build your workspace, as proposed in this [README](https://github.com/ros-planning/warehouse_ros_mongo):
```sh
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git
```
```sh
sudo apt-get install scons
```
```sh
cd mongo-cxx-driver
```
```sh
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
```
To use **qt_ros** correctly in your computer (necessary to build), please install QT Make:
```sh
sudo apt-get install qt4-default
```

Besides that, you need **MongoDB**. To install the complete version for Ubuntu 18.04, follow the steps bellow in a terminal:
```sh    
wget -qO - https://www.mongodb.org/static/pgp/server-4.2.asc | sudo apt-key add -
```
If you get an ERROR, try this:
```sh    
sudo apt-get install gnupg
```
```sh    
wget -qO - https://www.mongodb.org/static/pgp/server-4.2.asc | sudo apt-key add -
```
Else, continue:
```sh    
echo "deb [ arch=amd64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.2 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.2.list
```
```sh    
sudo apt-get update
```
```sh    
sudo apt-get install -y mongodb-org
```
For more info about MongoDB installation, acess [here](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/).

### Manual changes
Some packages presented in *Workspace Dependencies* required manual changes.

#### apriltag_ros
This wrapper is used to tag detection. For this repository, more precisely the developed packages that use a camera, you need to change the configuration files in ```config folder```:
- *settings.yaml*: Define ```tag_family: 'tag25h9'```. This option is related to the tag family used here.
- *tags.yaml*: Define in ```standalone_tags``` the tags that are going to be used. For this repository, was only necessary one tag that you define as:
```yaml
standalone_tags:
  [
    {id: 1, size: 0.15, name: tag00001}
  ]
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
