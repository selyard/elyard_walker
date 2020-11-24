## Installation

Note that this is dependant on all dependencies noted below.

```bash
cd ~/catkin_ws/src/
git clone https://github.com/selyard/elyard_walker.git
cd ~/catkin_ws && catkin_make
```

## Dependencies

This was developed using ROS Melodic and Gazebo. I was unable to install the Turtlebot simulator using the in-slide instructions. I found alternate installation instructions [on this site](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/), but they may produce a slightly different result than originally intended.

```bash
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

## Usage

### To Run Using Launch File

Parts of the code for this launch file was copied from the "turtlebot3_gazebo turtlebot3_world.launch" file included in the Turtlebot3 Simulation git source. It was modified to decrease the time before the rover encountered an obstacle.

This code could conceivably work in any Gazebo world; however, it was only tested on this specified world. This world contains an outside barrier preventing the robot from traveling into infinity, as well as a series of pillars which serve as obstacles to avoid.

```bash
roslaunch elyard_walker walker_demo.launch
```

There are several argument options for the launch file:
* Walker Algorithm:
  * freq:=(integer) defines a frequency of output messages; defaults to 30hz
  * rec:=true records a 30-second ROSbag; defaults to false. Recording is saved in ~/.ros/ directory.
  * avoid_angles:=(integer) define a number of rangefinder sensor intervals to avoid for collision avoidance - for the burger model, 1 interval is roughly 1 degree. This is plus/minus value from center. Defaults to 10.
  * min_dist:=(double) defines how close the robot can get to an obstacle before turning to avoid; defaults to 0.75.
* Gazebo Parameters
  * model:=(string) defines the rover model; defaults to "burger".
  * x_pos:=(double) defines the starting X position; defaults to (-2.0)
  * y_pos:=(double) defines the starting Y position; defaults to (0.0)
  * z_pos:=(double) defines the starting Z position; defaults to (0.0)

### Working with Recorded Bag File

The recorded bag file is stored in thedefault ~/.ros/ directory. To inspect the pre-recorded bagfile provided, starting from the install directory
```bash
cd results
rosbag info output_bag.bag
```

To play back the recorded bagfile, ROScore needs to be running as well.
```bash
roscore
```
Once this is running, from the install directory:
```bash
cd results
rosbag play output_bag.bag
```

### To Run Sections Individually

To run Gazebo:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

To tele-operate: in a separate terminal, run:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

To run the walker script instead, in a separate terminal, run:
```bash
rosrun elyard_walker walker_script
```

### Other Notes

#### Deliverables
Deliverables are included in the Results folder.

#### CPPCheck + CPPLint
CPPCheck and CPPLint checked and passed. To run the scripts, navigate to the install directory and run:

```bash
chmod +x check_cppcheck.sh check_cpplint.sh
./check_cppcheck.sh
./check_cpplint.sh
```

