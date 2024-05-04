# Fake Simulation

This package provides a simulation environment for a robot with a fake odometry and a fake laser scanner. It is useful for testing navigation planning algorithms without the need for a physical robot or a complex and heavy simulation environment.

## Components

The package consists of two main components:

1. **Fake Odometry**: This component simulates the odometry of a robot. It subscribes to a `cmd_vel` topic to receive velocity commands and uses dead reckoning to calculate the robot's pose. The pose is then published to an `odom` topic, with transform broadcasting. The initial pose of the robot can be set by publishing a `PoseWithCovarianceStamped` message to the `initialpose` topic, supported in Rviz toolbar button, `2D pose estimate`.

2. **Fake Laser**: This component simulates a laser scanner. It generates fake laser scans based on an occupancy grid map and the current pose of the robot. The laser scans are published to a `laser` topic.

## Dependencies

This package depends on the `map_server` ROS package.

## Usage

To use this package, you need to launch the `run.launch` file. This will start the fake odometry and fake laser nodes, as well as a map server and RViz for visualization.

```bash
roslaunch fake_simulation run.launch
```

The map server loads a map from a YAML file and publishes it to the `map` topic. The fake laser node subscribes to this topic to get the map.

## Configuration

The behavior of the fake odometry and fake laser nodes can be configured through ROS parameters. These parameters can be set in the launch file or through the command line when launching the nodes.

Here are some of the parameters:

- `cmd_vel`: The topic to subscribe to for velocity commands.
- `odom`: The topic to publish the odometry to.
- `initialpose`: The topic to subscribe to for setting the initial pose.
- `laser`: The topic to publish the laser scans to.
- `map`: The topic to subscribe to for the map.
- `baselinkFrame`: The frame ID of the robot's base.
- `odometryFrame`: The frame ID of the odometry.
- `mapFrame`: The frame ID of the map.
- `laserFrame`: The frame ID of the laser.

For a full list of parameters, please refer to the source code.