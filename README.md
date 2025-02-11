# Turtlebot Bookstore Sim

This package contains a simulator for a Turtlebot navigating under uncertainty in a book store.
Boxes appear randomly in the environment and can be cleared by the robot.

This simulator is used to evaluate [REFINE-PLAN](https://github.com/convince-project/refine-plan) from the paper **'Planning under Uncertainty from Behaviour Trees'**.

The environment in this repo is a modification of [The ROS2 branch of the AWS Robomaker Bookstore World](https://github.com/aws-robotics/aws-robomaker-bookstore-world/tree/ros2).

## Dependencies & Installation

This package has been tested in ROS 2 Humble.

First, install the following packages:
```
sudo apt install ros-humble-gazebo-* ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-dynamixel-sdk ros-humble-turtlebot3-msgs ros-humble-turtlebot3
```

Note that navigation 2 won't work out of the box with the turtlebot 3 in ROS 2 Humble.
To fix it, change robot_model_type: "differential" to "nav2_amcl::DifferentialMotionModel" in `/opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml`.

Additional dependencies:
* [Topological Navigation](https://github.com/HyPAIR/topological_navigation)
* [Topological Messages](https://github.com/HyPAIR/topological_msgs)
* [Door Messages](https://github.com/HyPAIR/door_msgs)
* [Shapely](https://pypi.org/project/shapely/) (Tested with version 2.0.6)
* [PyMongo](https://pymongo.readthedocs.io/en/stable/index.html) (Tested with 4.8.0)
* [REFINE-PLAN](https://github.com/convince-project/refine-plan)

After installing the dependencies, place this package into the `src` directory of your workspace and run `colcon build --symlink-install` to build.

## Replicating Experiments
To replicate the experiments in 'Planning under Uncertainty from Behaviour Trees', first set up a mongoDB instance either locally or on the cloud.

### Setting up a Local MongoDB instance
Follow the instructions [here](https://www.mongodb.com/docs/manual/tutorial/install-mongodb-on-ubuntu/).
After installation, run `sudo systemctl start mongod`.
The MongoDB instance will be at `localhost:27017`.

Then run:
```
cd bringup
./run_experiments.sh <mode> <MongoDB instance address>
```

The `<mode>` can be set to:
* `data` which runs 100 rounds of 100 random actions for data collection.
* `initial` which runs the initial BT 100 times.
* `refined` which runs the refined policy 100 times.

## Maintainer

This repository is maintained by:

| | | |
|:---:|:---:|:---:|
| Charlie Street | [@charlie1329](https://github.com/charlie1329) |[c.l.street@bham.ac.uk](mailto:c.l.street@bham.ac.uk?subject=[GitHub]%20Turtlebot%20Bookstore%20Sim)|
