# Robotic vehicle follower in ROS

The purpose of the project is the creation of algorithm in which a mobile robotic vehicle with 3 degrees of fredom can keep a specific distance from a wall as it moves alongside it following curves without crushing or leaning away. 

The current project has been createed with ROS and it was teasted in gazebo simulation environment. 

The robotic vehicle is equiped with 3 distanse sensors in the front with continiously updatable values. These values are fed as an input at a PID controller to provide as an output the linear and angular velocity. This way the robot can consistenly update its velocity and position due to keep "following" the obstacles along the way.
