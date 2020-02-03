# MoveIt! Package for OpenMANIPULATOR-PRO with RGB Camera and Gripper

Package to control the manipulator using a XBOX360 joystick with graphical visualization in python.

## **Requirements**

This repository was created using:
- ROS version - **Melodic**
- OS - **Ubuntu 18.04**

### ROS Packages
It is necessary to have some controllers packages:
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

## **Usage**
### **1. Joystick control in Gazebo simulation**

#### Run Gazebo
```sh
roslaunch bir_open_manipulator_p_joy_control bir_omp_joystick_simulation.launch
```

#### Run the JOYSTICK controller
```sh
rosrun bir_open_manipulator_p_joy_control bir_omp_joystick_simulation.py
```

#### Run the graphical visualization
```sh
rosrun bir_open_manipulator_p_joy_control bir_omp_plot_real_time.py
```

### **1. Joystick control in Real world**

#### Run Gazebo
```sh
roslaunch bir_open_manipulator_p_joy_control bir_omp_joystick_realWorld.launch
```

#### Run the JOYSTICK controller
```sh
rosrun bir_open_manipulator_p_joy_control bir_omp_joystick_realWorld.py
```

#### Run the graphical visualization
```sh
rosrun bir_open_manipulator_p_joy_control bir_omp_plot_real_time.py
```
