# TSUPORT Rescue Drone

Author: Hardefa Rogonondo

TSUPORT stands for Tsunami Post Disaster Robot. TSUPORT is a disaster robot system to help the search and rescue process especially after tsunami disaster. TSUPORT utilize the use of quadcopter drone and mobile robot simultaneously in post disaster area. TSUPORT drone is able to fly autonomously indoor or inside ruins of disaster thanks to the implemented features of the obstacle avoidance and path planning algorithm.
TSUPORT uses VFH+ algorithm
This repository contain 2 subject of the research, the simulation and the implementation.

The simulation documentation contains information about how to setup and run the two planner systems on the Gazebo simulator and on a companion computer running Ubuntu 18.04 (recommended), for both avoidance and collision prevention use cases.

PX4 computer vision algorithms packaged as ROS nodes for depth sensor fusion and obstacle avoidance.
This repository contains two different implementations:

  * *local_planner* is a local VFH+* based planner that plans (including some history) in a vector field histogram

The *local_planner* requires less computational power but it doesn't compute optimal paths towards the goal since it doesn't store information about the already explored environment. An in-depth discussion on how it works can be found in [this thesis](https://drive.google.com/open?id=1yjDtxRrIntr5Mdaj9CCB4IFJn0Iy2-bR). On the other hand, the *global_planner* is computationally more expensive since it builds a map of the environment. For the map to be good enough for navigation, accurate global position and heading are required. An in-depth discussion on how it works can be found in [this thesis](https://drive.google.com/open?id=1hhMLoXQuEM4ppdvDik8r6JY3RwGi5nzw).

# Table of Contents
- [TSUPORT Rescue Drone](#TSUPORT-Rescue-Drone)
  - [Installation](#installation)
    - [Installation for Ubuntu](#installation)
  - [Run the Avoidance Gazebo Simulation](#run-the-avoidance-gazebosimulation)
    - [Local Planner](#local-planner)
  - [Run on Hardware](#run-on-hardware)
    - [Prerequisite](#prerequisite)
    - [Local Planner](#local-planner)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

# Installation

## Simulation Installation

This is a step-by-step guide to install and build all the prerequisites for running the avoidance module on **Ubuntu 18.04** with *ROS Melodic* (includes Gazebo 9).
You might want to skip some steps if your system is already partially installed.

> **Note:** These instructions assume your catkin workspace (in which we will build the avoidance module) is in `~/catkin_ws`, and the PX4 Firmware directory is `~/Firmware`.
  Feel free to adapt this to your situation.

1. Add ROS to sources.list:
     ```bash
     sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
     sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
     sudo apt update
     ```

1. Install ROS with Gazebo:
     ```bash
     sudo apt install ros-melodic-desktop-full

     # Source ROS
     echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
     source ~/.bashrc
     ```
   > **Note** We recommend you use the version of Gazebo that comes with your (full) installation of ROS.
   >  If you must to use another Gazebo version, remember to install associated ros-gazebo related packages:
   >  - For Gazebo 8,
       ```sh
       sudo apt install ros-melodic-gazebo8-*
       ```
    > - For Gazebo 9,
       ```
       sudo apt install ros-melodic-gazebo9-*
       ```

1. Initialize rosdep.
   ```bash
   rosdep init
   rosdep update
   ```

1. Install catkin and create your catkin workspace directory.

   ```bash
   sudo apt install python-catkin-tools
   mkdir -p ~/catkin_ws/src
   ```

1. Install MAVROS (version 0.29.0 or above).
   > **Note:** Instructions to install MAVROS from sources can be found [here](https://dev.px4.io/en/ros/mavros_installation.html).

     ```bash
     sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
     ```

1. Install the *geographiclib* dataset

   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   chmod +x install_geographiclib_datasets.sh
   sudo ./install_geographiclib_datasets.sh
   ```

1. Install avoidance module dependencies (pointcloud library and octomap).
     ```bash
     sudo apt install libpcl1 ros-melodic-octomap-*
     ```

1. Clone this repository in your catkin workspace in order to build the avoidance node.
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/PX4/avoidance.git
   ```

1. Actually build the avoidance node.

   ```bash
   catkin build -w ~/catkin_ws
   ```

   Note that you can build the node in release mode this way:

   ```bash
   catkin build -w ~/catkin_ws --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

1. Source the catkin setup.bash from your catkin workspace:
   ```bash   
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Run the Avoidance Simulation

In the following section we guide you through installing and running a Gazebo simulation of local planner.

### Build and Run the Simulator

1. Clone the PX4 Firmware and all its submodules (it may take some time).

   ```bash
   cd ~
   git clone https://github.com/PX4/Firmware.git --recursive
   cd ~/Firmware
   ```

1. Install [PX4 dependencies](http://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies). 
   ```bash
   # Install PX4 "common" dependencies.
   ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
   
   # Gstreamer plugins (for Gazebo camera)
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev

1. Build the Firmware once in order to generate SDF model files for Gazebo.
   This step will actually run a simulation (that you can immediately close).

   ```bash
   # This is necessary to prevent some Qt-related errors (feel free to try to omit it)
   export QT_X11_NO_MITSHM=1

   # Build and run simulation
   make px4_sitl_default gazebo
   
   # Quit the simulation (Ctrl+C)

   # Setup some more Gazebo-related environment variables (modify this line based on the location of the Firmware folder on your machine)
   . ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
   ```

1. Add the Firmware directory to ROS_PACKAGE_PATH so that ROS can start PX4:
   ```bash
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
   ```
1. Finally, set the GAZEBO_MODEL_PATH in your bashrc:
   ```bash
   echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds" >> ~/.bashrc
   ```

The last three steps, together with sourcing your catkin **setup.bash** (`source ~/catkin_ws/devel/setup.bash`) should be repeated each time a new terminal window is open.
You should now be ready to run the simulation using local or global planner.

### Local Planner (default, heavily flight tested)

This section shows how to start the *local_planner* and use it for avoidance in mission or offboard mode.

The planner is based on the [3DVFH+](http://ceur-ws.org/Vol-1319/morse14_paper_08.pdf) algorithm.

> **Note:** You *may* need to install some additional dependencies to run the following code (if not installed):
>   ```sh
>   sudo apt install ros-melodic-stereo-image-proc ros-melodic-image-view
>   ```

Any of the following three launch file scripts can be used to run local planner:
> **Note:** The scripts run the same planner but simulate different sensor/camera setups. They all enable *Obstacle Avoidance* and *Collision Prevention*.
* `local_planner_stereo`: simulates a vehicle with a stereo camera that uses OpenCV's block matching algorithm (SGBM by default) to generate depth information
  ```bash
  roslaunch local_planner local_planner_stereo.launch
  ```
    
  > **Note:** The disparity map from `stereo-image-proc` is published as a [stereo_msgs/DisparityImage](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html) message, which is not supported by rviz or rqt. 
  > To visualize the message, first open a *new terminal* and setup the required environment variables:
  > ```bash
  > source devel/setup.bash
  > ```
  > Then do either of:
  > - run:
  >   ```bash
  >   rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
  >   ```
  > - publish the `DisparityImage` as a simple `sensor_msgs/Image`:
  >   ```bash
  >   rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image 'm.image' 
  >   ```
  > The disparity map can then be visualized by *rviz* or *rqt* under the topic */stereo/disparity_image*.

* `local_planner_depth_camera`: simulates vehicle with one forward-facing kinect sensor
  ```bash
  roslaunch local_planner local_planner_depth-camera.launch
  ```

* `local_planner_sitl_3cam`: simulates vehicle with 3 kinect sensors (left, right, front)
  ```bash
  roslaunch local_planner local_planner_sitl_3cam.launch
  ```

You will see the Iris drone unarmed in the Gazebo world.
To start flying, there are two options: OFFBOARD or MISSION mode.
For OFFBOARD, run:

```bash
# In another terminal 
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

The drone will first change its altitude to reach the goal height.
It is possible to modify the goal altitude with `rqt_reconfigure` GUI.
![Screenshot rqt_reconfigure goal height](docs/lp_goal_height.png)
Then the drone will start moving towards the goal.
The default x, y goal position can be changed in Rviz by clicking on the 2D Nav Goal button and then choosing the new goal x and y position by clicking on the visualized gray space.
If the goal has been set correctly, a yellow sphere will appear where you have clicked in the grey world.
![Screenshot rviz goal selection](docs/lp_goal_rviz.png)

For MISSIONS, open [QGroundControl](http://qgroundcontrol.com/) and plan a mission as described [here](https://docs.px4.io/en/flight_modes/mission.html). Set the parameter `COM_OBS_AVOID` true.
Start the mission and the vehicle will fly the mission waypoints dynamically recomputing the path such that it is collision free.



Initially the drone should just hover at 3.5m altitude.

From the command line, you can also make Gazebo follow the drone, if you want.

```bash
gz camera --camera-name=gzclient_camera --follow=iris
```

One can plan a new path by setting a new goal with the *2D Nav Goal* button in rviz.
The planned path should show up in rviz and the drone should follow the path, updating it when obstacles are detected.
It is also possible to set a goal without using the obstacle avoidance (i.e. the drone will go straight to this goal and potentially collide with obstacles). To do so, set the position with the *2D Pose Estimate* button in rviz.

# Troubleshooting

## Simulation

### I see the drone position in rviz (shown as a red arrow), but the world around is empty
Check that some camera topics (including */camera/depth/points*) are published with the following command:

```bash
rostopic list | grep camera
```

If */camera/depth/points* is the only one listed, it may be a sign that gazebo is not actually publishing data from the simulated depth camera. Verify this claim by running:

```bash
rostopic echo /camera/depth/points
```

When everything runs correctly, the previous command should show a lot of unreadable data in the terminal. If you don't receive any message, it probably means that gazebo is not publishing the camera data.

Check that the clock is being published by Gazebo:

```bash
rostopic echo /clock
```

If it is not, you have a problem with Gazebo (Did it finish loading the world? Do you see the buildings and the drone in the Gazebo UI?). However, if it is publishing the clock, then it might be a problem with the depth camera plugin. Make sure the package `ros-kinetic-gazebo-ros-pkgs` is installed. If not, install it and rebuild the Firmware (with `$ make px4_sitl_default gazebo` as explained above).

### I see the drone and world in rviz, but the drone does not move when I set a new "2D Nav Goal"
Is the drone in OFFBOARD mode? Is it armed and flying?

```bash
# Set the drone to OFFBOARD mode
rosrun mavros mavsys mode -c OFFBOARD
# Arm
rosrun mavros mavsafety arm
```

### I see the drone and world in rviz, but the drone does not follow the path properly
Some tuning may be required in the file *"<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris"*.

### I see the drone and world in rviz, I am in OFFBOARD mode, but the planner is still not working
Some parameters that can be tuned in *rqt reconfigure*.

# Contributing

Fork the project and then clone your repository. Create a new branch off of master for your new feature or bug fix.

Commit your changes with informative commit messages, push your branch, and open a new pull request. Please provide ROS bags for the simulation and autopilot flight logs for the prototype robot relevant to the changes you have made.