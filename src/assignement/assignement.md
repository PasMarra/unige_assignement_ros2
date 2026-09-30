
- (1) allows the user to drive the robot around, by setting a linear and angular velocity;
- (3) allows the user to change the threshold, by means of a ROS service;
- (5) allows the user to get the average linear and angular velocity of the most recent 5 inputs, by means
of another ROS service.

- (2) if the user’s input causes the robot to be “too close” to one of the obstacles (e.g., the minimum value
of the laser scanner is below a certain threshold) moves the robot back to the previous position, to
remain in a safe area;
- (4) publishes on a topic a custom message with the distance of the closest obstacle, the direction of the
obstacle (e.g., left, front, right) and the threshold.


ROS service: (1), (3), (5)
ros node: (2), (4)

Topics:
(1) /cmd_vel
(2) /scan
