# TurtleSim-ROS

## Installation

To use this project, you will need to have ROS (Robot Operating System) installed on your system. You can follow the official ROS installation guide for your specific operating system: [ROS Installation Guide](http://wiki.ros.org/ROS/Installation).

Once you have ROS installed, you can clone the repository and build the package:

```
git clone https://github.com/rm-rf-humans/TurtleSim-ROS.git
cd TurtleSim-ROS
```

Set up the ROS workspace:
```
mkdir -p ~/catkin_ws/src
mv TurtleSim-ROS ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


## Usage

To run the bouncing turtle simulation, use the following command:

```
roscore
rosrun turtlesim turtlesim_node
rosrun TurtleSim-ROS bouncing_turtle.py
```

This will start the turtle simulation and display the turtle's trajectory on a plot.

## API

The `AdvancedTurtleBouncer` class in the `bouncing_turtle.py` file provides the following methods:

- `pose_callback(self, pose)`: Callback function for the turtle's pose topic, which updates the current pose of the turtle.
- `adjust_bounce_angle(self)`: Adjusts the bounce angle of the turtle based on its current position relative to the boundaries.
- `move(self)`: Moves the turtle by publishing velocity commands to the `/turtle1/cmd_vel` topic.
- `update_plot(self, frame)`: Updates the plot of the turtle's trajectory.
- `start_plot(self)`: Starts the plot in a separate thread.
- `run(self)`: Runs the bouncing turtle simulation.
  

## Dependencies

- ROS Noetic
- `matplotlib` for plotting
- `geometry_msgs` for handling turtle movement
- `turtlesim` for the turtle simulation

You can install the necessary Python dependencies using:

```
pip install matplotlib
```
