# Turtlebot Bookstore Sim

This package contains a simulator for a turtlebot acting in a bookstore environment.
Boxes appear randomly in the environment and can be cleared by the robot.
This simulation is primarily used for evaluating [REFINE-PLAN](https://github.com/convince-project/refine-plan) in CONVINCE.

The sim in this repo modifies the simulation in [The ROS2 branch of the AWS Robomaker Bookstore World](https://github.com/aws-robotics/aws-robomaker-bookstore-world/tree/ros2) - this package is not used directly, but its dependencies must be installed to install all world assets.


Navigation 2 won't work out of the box with the turtlebot 3 - change robot_model_type: "differential" to "nav2_amcl::DifferentialMotionModel" in `/opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml`
