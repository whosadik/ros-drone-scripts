# ROS Drone Scripts

A collection of Python scripts to control a PX4 drone using ROS and MAVROS.

## Scripts

- `fly_square.py`: Fly in a square pattern.
- `fly_to_target.py`: Fly through custom input waypoints.
- `follow_target.py`: Follow dynamic target from `/target_position` topic.
- `target_generator.py`: Continuously publishes changing target positions.

## Requirements

- ROS Noetic / ROS 1
- MAVROS
- PX4 SITL or real drone setup

## Usage

Run in a catkin workspace:

```bash
rosrun flybot scripts/fly_square.py
