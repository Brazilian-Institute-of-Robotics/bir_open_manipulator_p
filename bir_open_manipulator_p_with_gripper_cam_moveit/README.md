# MoveIt! Package for OpenMANIPULATOR-PRO with RGB Camera and Gripper

Package developed to present a physical-simulate control for OpenMANIPULATOR-PRO through MoveIt! with:
- **RGB camera** (model Teledyne Nano Series)
- **custom gripper**

For several tasks

![omp_w_camera_gripper](https://user-images.githubusercontent.com/32513366/71099457-15a07300-2192-11ea-918d-7a39e6989054.png)

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
If you don't have a catkin workspace, create a new workspace:
```sh
mkdir -p ~workspaces/omp_ws/src
cd ~/workspaces/omp_ws/src
```
Beside this meta package itself:
- [bir_open_manipulator_p](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p.git) (master branch)
    ```sh
    git clone -b master https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p.git
    ```

Install the dependencies that are necessary to use OpenManipulator-PRO and simulations with ROS/Gazebo (only if you don't have in your workspace):

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
This wrapper is used to tag detection. For this application, you need to change the configuration files in ```config folder```:
- *settings.yaml*: Define ```tag_family: 'tag25h9'```. This option is related to the tag family used here.
- *tags.yaml*: Define in ```standalone_tags``` the tags that are going to be used. Following the example presented in this file, it is necessary to add one tag that you define as:
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
Or if you already have a workspace, in a terminal:
```sh
catkin build
rosdep install --from-paths src --ignore-src -r -y
```
Those commands install and update any missing requirement for your packages.

## **Requirements for Teledyne RGB camera**
The camera used for this challenge was a [Teledyne Nano Series](https://www.teledynedalsa.com/en/products/imaging/cameras/genie-nano-family/). It is a little bit tricky to use it, so I recommend:
- Follow the tutorial present in this [README](https://github.com/Brazilian-Institute-of-Robotics/def_cam_teledyne_nano). Note that it is necessary to install [DALSA Framework](https://www.teledynedalsa.com/en/products/imaging/vision-software/linux-gige-v-framework/).
- The Ethernet connection needs to be **Link-Local Only** for IPv4. To change it, go to *Wired Settings* in Ubuntu System.

## **Create your own apriltags**
The process follows:

**1.** Clone/Download this [repository](https://github.com/AprilRobotics/apriltag-imgs). It contains created tags for every family in April tag. 

**2.** Select one tag from any family, was used *tag25h9* for this challenge and using **GIMP** rescale the tag for your desired size.

**NOTE**: It is important to use one family only, the apriltag_ros package does not allow two or more tags from different families for recognition at the same time

**3.** In **apriltag_ros** package it is necessary to do some changes in config folder
- In *settings.yaml* change your tag family
- In *tags.yaml* insert your tag in standalone_tags following the example present in the same file

**4.** Follow this [tutorial](https://www.youtube.com/watch?v=WDhIaVOUwsk) to insert a tag in your simulation. If you are going to real world you can print your tag and put in front of the camera.

## **Package Uses**
### **1. Catch with Apriltag Dectection**
This challenge propose to search for a tag and after the detection move to a specific position, grap an object and drop it in other specific position.

![simulation](https://user-images.githubusercontent.com/32513366/71099879-c6a70d80-2192-11ea-8142-60ff207adff3.png)

#### Simulation
- Run Gazebo with MoveIt!
```sh
roslaunch bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_simulation_for_apriltag_detection.launch
```
- Run the robot routine
```sh
rosrun bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_catch_simulation.py 
```
- Verify your tag detection

    To see where your camera found the tag in live stream there are two options:
    - Using ```image_view```. In a terminal:
    ```sh
    rosrun image_view image_view image /image:=/tag_detections_image
    ```
    - Using ```rqt_image_view```. Search for the topic ```/tag_detections_image``` after in a terminal:
    ```sh
    rqt_image_view
    ```
   
#### **Real world**

- Run Robot Controllers with MoveIt! background
```sh
roslaunch bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_deploy_robot_with_moveit.launch
```
- Run Camera and Apriltag Detection
```sh
roslaunch bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_deploy_cam_for_apriltag_detection.launch 
```
- Run the robot routine
```sh
rosrun bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_catch_realWorld2.py 
```
- Verify your tag detection

    To see where your camera found the tag in live stream there are two options:
    - Using ```image_view```. In a terminal:
    ```sh
    rosrun image_view image_view image /image:=/tag_detections_image
    ```
    - Using ```rqt_image_view```. Search for the topic ```/tag_detections_image``` after in a terminal:
    ```sh
    rqt_image_view
    ```

### **2. Move end effector in XYZ**
This task propose to movement the gripper in X, Y or Z through joystick, creating a response graph.

#### Rviz
- Initialize joystick communication
```
sudo xboxdrv --detach-kernel-driver
```
- Run RViz
```
roslaunch bir_open_manipulator_p_with_gripper_cam_moveit demo.launch
```
- Run Joystick node
```
rosrun joy joy_node
```
- Run Response graph

    There are two options for response graph: **time** and **command**. In the first option will have a response with X-axis in time, in the second will have a response with X-axis in commands (related to how many commands you send to OpenManipulator-PRO).

    - Run response graph in **time**
        ```
        rosrun bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_plot_end_effector_xyz_time.py
        ```
    - Or run response graph in **command**
        ```
        rosrun bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_plot_end_effector_xyz_command.py
        ```

- Run task routine
```
rosrun bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_end_effector_xyz_joystick_control.py
```

### **3. MoveIt! Benchmark for Developed model**
This task evaluate a series of planners from OMPL library for the given URDF model.

#### Benchmark
- Run the process
```
roslaunch bir_open_manipulator_p_with_gripper_cam_moveit bir_omp_benchmark.launch
```
And wait for the results (depends on your config, the process can take a long time to finish).

**PS**: For more info related to how to change benchmark configuration, scenes, robot positions and generate graphical results acess this [README](https://github.com/Brazilian-Institute-of-Robotics/bir_open_manipulator_p/tree/master/bir_open_manipulator_p_moveit) that contains all process related to develop a Benchmark evaluation.

## **Package Notes**
- Until now, to control physically the gripper is necessary to use a **service/client** and not MoveIt!. However, in simulation MoveIt! can control the gripper properly.
- If you see any strange movement in your manipulator during simulation, note that this simulation is using PID controller in ros_control.
