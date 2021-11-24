# Robotic vehicle follower in ROS

This project contains an algorithm with which a mobile vehicle with 3 degrees of fredom can keep a specific distance from a wall, as it moves alongside it following curves without crushing or leaning away. 

The current project has been createed with ROS and it was tested in gazebo simulation environment. 

The robotic vehicle is equiped with 3 distanse sensors in the front with continiously updatable values. These values consist the input of a PID controller which provides as output the linear and angular velocity of the vehicle. This way the robot can consistenly update its velocity and position due to keep "following" the obstacles without ever crushing.

# License
https://choosealicense.com/licenses/mit/
