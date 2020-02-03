# **BIR OpenMANIPULATOR-PRO**

FOTO - MANIPULADOR ALTERADO

Explicar que tudo que foi criado tem a tag bir_omp (pacotes e codigos)

[ROBOTIS README](https://github.com/ROBOTIS-GIT/open_manipulator_p)

## **Summary**
- Modifications
- Requirements
- Dynamixel Wizard 2.0
- Developed ROS Packages

## **Modifications**

## **Requirements**

### System Requirements
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
sudo apt-get install ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-controller-manager ros-melodic-joint-trajectory-controller ros-melodic-joint-state-controller
ros-melodic-position-controllers
```
It is strictly recommend to have those specifications, before you proceed.

### OpenManipulator-PRO dependencies
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

### Repository dependencies
To use correctly any package developed here, get those dependencies:
- [apriltag](https://github.com/AprilRobotics/apriltag) (master branch)
    ```sh
    git clone -b master https://github.com/AprilRobotics/apriltag.git
    ```
- [apriltag_ros](https://github.com/AprilRobotics/apriltag) (master branch)
    ```sh
    git clone -b master https://github.com/AprilRobotics/apriltag_ros.git
    ```
- [warehouse_ros_mongo](https://github.com/ros-planning/warehouse_ros_mongo.git) (melodic-devel branch)
    ```sh
    git clone -b melodic-devel https://github.com/ros-planning/warehouse_ros_mongo.git
    ```
- [qt_ros](https://github.com/stonier/qt_ros) (indigo branch)
    ```sh
    git clone -b indigo https://github.com/stonier/qt_ros.git
    ```
- [def_cam_teledyne_nano](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano) (feature-flip branch)
    ```sh
    git clone -b feature-flip https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano.git
    ```


And **MongoDB**. To install the complete version for Ubuntu 18.04, follow the steps bellow in a terminal:
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


### Manual changes
Some packages presented in *Repository Dependencies* required manual changes.

#### apriltag_ros
This wrapper is used to tag detection. For this repository, more precisely the developed packages that use a camera you need to change configuration files:
- *settings.yaml*: Define ```tag_family: 'tag25h9'```. This option is related to the tag family used here.
- *tags.yaml*: Define in ```standalone_tags``` the tags that are going to be used. For this repository, was only necessary one tag that you define as:
    ```mak
    standalone_tags:
    [
        {id: 1, size: 0.15, name: tag00001}
    ]
    ```




## **Dynamixel Wizard 2.0**

## **Developed ROS packages**
