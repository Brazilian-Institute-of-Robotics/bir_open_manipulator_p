# **BIR OpenMANIPULATOR-PRO**

![ompBIR](https://user-images.githubusercontent.com/32513366/73663829-15deca00-467d-11ea-853e-bce2171e91f0.jpeg)

Repository for several uses with OpenManipulator-PRO from ROBOTIS customized in BIR (Brazilian Institute of Robotics). This content was first forked from an original repository, for more information access [here](https://github.com/ROBOTIS-GIT/open_manipulator_p).

## **Summary**
- Robot Modifications
- Requirements
- Dynamixel Wizard 2.0
- Developed ROS Packages

## **Robot Modifications**
Was added to OpenManipulator-PRO two itens:
- Custom Gripper
- Camera

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
sudo apt-get install joy ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-controller-manager ros-melodic-joint-trajectory-controller ros-melodic-joint-state-controller
ros-melodic-position-controllers
```
It is strictly recommend to have those specifications, before you proceed.

### OpenManipulator-PRO Dependencies
First, to use this meta package create a new workspace:
```sh
mkdir -p ~workspaces/omp_ws/src
cd ~/workspaces/omp_ws/src
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
- [def_cam_teledyne_nano](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano) (feature-flip branch)
    ```sh
    git clone -b feature-flip https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano.git
    ```

- [warehouse_ros_mongo](https://github.com/ros-planning/warehouse_ros_mongo.git) (melodic-devel branch)
    ```sh
    git clone -b melodic-devel https://github.com/ros-planning/warehouse_ros_mongo.git
    ```
- [qt_ros](https://github.com/stonier/qt_ros) (indigo branch)
    ```sh
    git clone -b indigo https://github.com/stonier/qt_ros.git
    ```

Besides that, you need **MongoDB**. To install the complete version for Ubuntu 18.04, follow the steps bellow in a terminal:
```sh    
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 2930ADAE8CAF5059EE73BB4B58712A2291FA4AD5
```
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
        {id: 1,size: 0.15,name: tag00001}
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

**PS**: You can use this software without any ROS connection. But, if you use this software, you can not use at the same time any ROS package to communicate with your robot.

## **Developed ROS packages**

### MoveIt! Planning Benchmark
- **Overview**: Package to evaluate planners efficiency for OpenManipulator-PRO

![scene](https://user-images.githubusercontent.com/32513366/69746813-db5c2c80-1123-11ea-86cd-9a9bcba4f18a.png)

- **ROS package**: bir_open_manipulator_p_moveit
- **Workspace Depedencies**
    - warehouse_ros_mongo
    - qt_ros
- **Access link**: [README]()

### Camera RGB package for Apriltag Detection
- **Overview**: Package developed using MoveIt! to search and detect apriltags using a RGB camera

![omp_simulation](https://user-images.githubusercontent.com/32513366/71183468-cc662700-2256-11ea-9dc7-70dee951ed8a.png)

- **ROS package**: bir_open_manipulator_p_with_cam_moveit

- **Workspace Depedencies**
    - apriltag
    - apriltag_ros
    - def_cam_teledyne

- **Access link**: [README]()


### Gripper with Camera package for detection tasks
- **Overview**: Package developed using MoveIt! and can be considered an extension of previous package, but now was insert a custom gripper for several tasks based on tag detection

![omp_w_camera_gripper](https://user-images.githubusercontent.com/32513366/71099457-15a07300-2192-11ea-918d-7a39e6989054.png)

- **ROS package**: bir_open_manipulator_p_with_gripper_cam_moveit
- **Workspace Depedencies**
    - apriltag
    - apriltag_ros
    - def_cam_teledyne

- **Access Link**: [README]()

### Joy Control for OpenManipulator-PRO
- **Overview**: Package to control the manipulator using a XBOX360 joystick with graphical visualization in python.

- **ROS package**: bir_open_manipulator_p_joy_control

- **Workspace Depedencies**
    - None

- **Access Link**: [README]()
