# RoCoPAR: Robot Cooperative Perception and Response
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
<!-- [![Build Status](https://travis-ci.com/username/repo.svg?branch=main)](https://travis-ci.com/ICAWang/RoCoPAR) -->

<!-- Robot Cooperative Perception and Reactive Planning -->
<!-- Robust Collaborative Perception and Autonomous Response -->

## Introduction
Robot cooperative perception and response is useful in real scenarios,
especially in search and rescue scenarios.

**Rocopar** is developed on `Ubuntu 20.04` and `ROS Noetic`, which is inspired by the following projects:

> https://github.com/Icheler/coExplore

> https://github.com/MRSD-Team-RoboSAR/robosar

> https://github.com/HauserDong/hybrid_control_match/tree/master



## Development
- **Stage one**(now): In ros_stage, leverage a single robot to explore the whole map autonomously, and recognize the target near itself:
    > Deadline: May 22
  - Explore the unknown workspace via [frontier_exploration](http://wiki.ros.org/frontier_exploration)
  - Explore the unknown workspace via [rrt_exploration](http://wiki.ros.org/rrt_exploration)
  - Recognize target boxes and publish to the screen.
  - Extend to multi-robot scenarios.
  
- **Stage two**: In ros_stage, leverage multiple robots to explore the whole map autonomously, and react to the triggered tasks:
    > Deadline: May 30



- **Stage three**: In gazebo, leverage a fleet of robots to explore the whole map, while recognizing all targets;
    > Deadline: June 10

- **Stage four**: In gazebo, explore and response to the triggered tasks.
    > Deadline: June 20





## Features
- Cooperative perception using multi-sensor data fusion 
- Reactive planning algorithms for dynamic environments
- Task allocation and coordination among multiple robots
- Integration with ROS for ease of use and extensibility


## Installation and Implementation

### Prerequisites

- ROS Noetic
- Python 3.8+
- Install dependencies:

