# MoveIt! Package for OpenMANIPULATOR-PRO with RGB Camera

The package developed to simulate and control OpenMANIPULATOR-PRO with a teledyne for april tag detection using MoveIt!

![omp_simulation](https://user-images.githubusercontent.com/32513366/71183468-cc662700-2256-11ea-9dc7-70dee951ed8a.png)

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
sudo apt-get install ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-controller-manager ros-melodic-joint-trajectory-controller ros-melodic-joint-state-controller
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
- [def_cam_teledyne_nano](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano) (feature_flip branch)
    ```sh
    git clone -b feature_flip https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano.git
    ```

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
## **Requirements for Teledyne Camera**
The camera used for this challenge was a [Teledyne Nano Series](https://www.teledynedalsa.com/en/products/imaging/cameras/genie-nano-family/). It is a little bit tricky to use it, so I recommend:
- Follow the tutorial present in this [README](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano). Note that it is necessary to install [DALSA Framework](https://www.teledynedalsa.com/en/products/imaging/vision-software/linux-gige-v-framework/).
- The Ethernet connection needs to be **Link-Local Only** for IPv4. To change it, go to *Wired Settings* in your system


## **Usage**
### **1. April tag Challenge in Simulation**

#### Run Background
```sh
roslaunch bir_open_manipulator_p_with_cam_moveit bir_omp_deploy_simulation_for_apriltag_detection.launch
```
#### Run the challenge routine
```sh
rosrun bir_open_manipulator_p_with_cam_moveit bir_omp_apriltag_routine_simulation.py 
```

### **2. April tag Challenge in Real world**

#### Run Robot Controllers and MoveIt!
```sh
roslaunch bir_open_manipulator_p_with_cam_moveit bir_omp_deploy_robot_with_moveit.launch
```
#### Run Camera and Tag detection
```sh
roslaunch bir_open_manipulator_p_with_cam_moveit bir_omp_deploy_cam_for_apriltag_detection.launch
```

#### Run the challenge routine
```sh
rosrun bir_open_manipulator_p_with_cam_moveit bir_omp_apriltag_routine.py 
```

## **Create your own apriltags**
The process follows:

**1.** Clone/Download this [repository](https://github.com/AprilRobotics/apriltag-imgs). It contains created tags for every family in April tag. 

**2.** Select one tag from any family, was used *tag25h9* for this challenge and using **GIMP** rescale the tag for your desired size.

**NOTE**: It is important to use one family only, the apriltag_ros package does not allow two or more tags from different families for recognition at the same time

**3.** In **apriltag_ros** package it is necessary to do some changes in config folder
- In *settings.yaml* change your tag family
- In *tags.yaml* insert your tag in standalone_tags following the example present in the same file

**4.** Follow this [tutorial](https://www.youtube.com/watch?v=WDhIaVOUwsk) to insert a tag in your simulation. If you are going to real world you can print your tag and put in front of the camera.

