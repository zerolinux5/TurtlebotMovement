# TurtlebotMovement

A simple Joy to Turtlebot 4 Movement node in C++ 20.
The expectation is that a controller is already publishing on the `\joy` topic and the Turtlebot 4 receives commands in the `/cmd_vel` topic. The LiDAR is used to ensure the robot doesn't drive forward into obstacles.

To build
```
colcon build
```
To then run use the following launch file:

```
ros2 launch tb_movement tb_launch.py
```