# MoveIt! Package for Benchmark Analysis

The package developed to do planners benchmark for OpenManipulator-PRO using MoveIt!

![scene](https://user-images.githubusercontent.com/32513366/69746813-db5c2c80-1123-11ea-86cd-9a9bcba4f18a.png)

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

### Workspace dependencies
To use correctly any package developed here, get those dependencies in your workspace:

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

## **MoveIt! Benchmark**
MoveIt! Benchmarking helps the researcher to define the best planner trajectory given your parameters, environment and goals. To a better comprehension of this tool, some terms needs to be clarify:
- **planner**: Algorithm capable to calculate a path (trajectory) between two points.
- **start state**: Position where your robot start at the very beginning. Used to avoid any colision scenario.
- **scene**: It is your environment itself, can contains your robot and obstacles.
- **query**: A path defined between two points. Beside that, a query have information about the scene where it was set, i.e. a query is contain in a scene. 
- **runs**: Numeric parameter. Is the number of attempts that a planner will have to calculate a query's trajectory.
- **timeout**: Numeric parameter. Is the maximum time (in seconds) allowed for a planner to find a solution.

For more information about it and others tutorials, you can acess
[here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/benchmarking/benchmarking_tutorial.html).

### **1. Setting your environment for a Benchmark**

Run the **demo.launch**:

```sh
roslaunch bir_open_manipulator_p_moveit demo.launch db:=true
```

#### **1.1. Create your Scene**
Here, you will use RViz to create your scenes, queries and start states. The steps here was already created before, **so it is only necessary if you want to create your own tests**, if it is not the case go to **section 1.2.**

Following the steps in *MotionPlanning*:
1. In *Context tab*, click on **Connect** to create a link with your database created by MongoDB.
2. In *Scene Objects tab*, you can upload **.stl** files that will work as obstacles where you have the option to scale and change it positions in the grid. After that, click on **Publish Scene**.
3. After defining your obstacles in the grid, go to *Stored Scenes tab* and save your scene clicking in **Save Scene**. Double click to rename it.
4. To create a query, define your start and end point in *Planning tab*, go to *Stored Scenes tab*, click on your **sceneName** and then click on **Save Query**. Double click to rename it.
5. To save a start state, define a Start State in *Planning tab*, go to *Stored States* and then click on **Save Start**.

#### **1.2. Load Created Scene**
This package has a created scene within queries and start states. To load it, follow this steps in *MotionPlanning*:
1. In *Context tab*, click in **Connect** to create a link with your database created by MongoDB.
2. In *Stored Scenes tab*, click on **benchmark_scene** (or the name that you saved) and then click on **Load Scene**
3. To load a query from this scene, click on the **left arrow** in benchmark_scene and click in one of the queries that appears. After that click on **Load Query**.

Here you can see the presented scene with a specific query loaded, where the ending goal is in red.
![query with scene](https://user-images.githubusercontent.com/32513366/73451911-730a1100-4347-11ea-860e-b284187fb844.png)


### **1.3. Setting your Benchmark options**
To change any configuration detail for your benchmark, modify **bir_omp_benchmark_settings.yaml** in **config** folder. There, you will find those sections:
- **warehouse**: Default, related to MongoDB connection
- **parameters**: Here you can set the parameters for the process itself as *runs*, *timeout*, *queries and start states names*, *moveGroup name* and path definition to save(*output_directory*)
- **planner**: The set of algorithms that you want to eval.

**PS**: Probably a bug from MoveIt! Benchmarking, but all the queries will be evaluated regardless the queries that you set in **parameters**

### **2. Run the Benchmark**

```sh
roslaunch bir_open_manipulator_p_moveit bir_omp_benchmark.launch
```

This process can take a while to finish. After closing, will generate a series of *log files* related to specific queries in your *output_directory*.

### **2.1. Generate your Databases for further analysis**

```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py <diretorio_do_log>
```
Example:
```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py /tmp/moveit_benchmarks/omp/logFileName.log
```
Where **<diretorio_do_log>** is the path (where the created log is located, defined in *benchmark_settings.yaml*) for one of your logs created after the benchmark. Every query generate one log file.

### **2.2. Plot Results**
If you want to plot the results, you can use this command:

```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py -p <plotName> <diretorio_do_log>
```

Example:
```sh
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py -p fileSaveLocation /tmp/moveit_benchmarks/omp/logFileName.log
```

**PS**: It is necessary to rename manually the **.db** generated in your *home directory*. 
**PS2**: To improve your plots visual, please use moveit package from **melodic-devel branch** (not the master branch) before using moveit_ros_benchmark.

### **2.3. Analyze results**
Acess [Planner Arena](http://plannerarena.org/) and update your database **.db** to see even more results!

### Doubts
For any doubt, you can get in contact through the github **issue** channel or email.
