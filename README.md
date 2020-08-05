# TSUPORT Rescue Drone

Author: Hardefa Rogonondo

TSUPORT stands for Tsunami Post Disaster Robot. TSUPORT is a disaster robot system to help the search and rescue process especially after tsunami disaster. TSUPORT utilize the use of quadcopter drone and mobile robot simultaneously in post disaster area. TSUPORT drone is able to fly autonomously indoor or inside ruins of disaster thanks to the implemented features of the obstacle avoidance and path planning algorithm.
TSUPORT uses VFH+ algorithm
This repository contain 2 subject of the research, the simulation and the implementation.

The simulation documentation contains information about how to setup and run the two planner systems on the Gazebo simulator and on a companion computer running Ubuntu 18.04 (recommended), for both avoidance and collision prevention use cases.

# software requirement
Python 2.7
DroneKit
MAVlink
OpenCV
Mono (Optional)

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
