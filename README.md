# TurtlebotMovement

A simple Joy to Turtlebot 4 Movement node in C++ 20.
The expectation is that a controller is already publishing on the `\joy` topic and the Turtlebot 4 receives commands in the `/cmd_vel` topic

To build
```
colcon build
```
To then run
```
ros2 run tb_movement tb_movement
```