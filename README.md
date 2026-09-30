# Assignement

- allows the user to drive the robot around, by setting a linear and angular velocity;
- if the user’s input causes the robot to be “too close” to one of the obstacles (e.g., the minimum value
of the laser scanner is below a certain threshold) moves the robot back to the previous position, to
remain in a safe area;
- allows the user to change the threshold, by means of a ROS service;
- publishes on a topic a custom message with the distance of the closest obstacle, the direction of the
obstacle (e.g., left, front, right) and the threshold.
- allows the user to get the average linear and angular velocity of the most recent 5 inputs, by means
of another ROS service.

# Create ROS2 workspace

Create and install the jazzy environment:

```terminal
pixi install -e jazzy
```

Build pkgs:

```terminal
pixi run -e jazzy build

```

# Launch the simulation env

Enter the pixi environment and launch the simulation environment:

```terminal
pixi shell -e jazzy
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

# Launch the assignement

Enter the pixi environment and launch the assignement:

```terminal
pixi shell -e jazzy
ros2 launch assignement assignement.launch.py
```
