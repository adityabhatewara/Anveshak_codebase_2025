# Team Anveshak: ROS2 Codebase 2025

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue.svg)](https://www.python.org/)

The official software stack for **Team Anveshak**, the space robotics team representing IIT Madras. This repository contains the ROS2 workspace and core packages responsible for the autonomous navigation, computer vision, and systems control of our Rover and Drone platforms for the International Rover Challenge.

## Core Capabilities
This package handles the high-level intelligence of the systems:
* **Computer Vision:** Real-time object detection and spatial awareness using **YOLO**.
* **Mapping & Localization:** Integration with **RTAB-Map** for 3D environment mapping and accurate rover/drone positioning.
* **Path Planning:** Autonomous trajectory generation and obstacle avoidance.
* **Custom Messaging:** Defined ROS2 `.msg` interfaces for seamless communication between the vision, planning, and hardware-control nodes.

## Repository Structure
Structured as a standard ROS2 package, containing Python-based nodes and custom launch files.

* `scripts/`: The core Python executables (ROS2 nodes) handling vision, path planning, and logic.
* `launch/`: ROS2 launch files to spin up multiple nodes and configure parameters simultaneously.
* `msg/`: Custom message definitions for specific telemetry and coordinate data.
* `previous_versions/`: Archived algorithms and deprecated scripts for reference.
* `CMakeLists.txt` & `package.xml`: ROS2 package configuration and dependency tracking.

## System Architecture (Code Flow)
The following diagram illustrates the ROS2 node communication and topic structure within this package:

```mermaid
graph TD
    %% Define Nodes
    Cam[Camera/Sensors] -->|Raw Image/Depth| CV[Vision Node: YOLO]
    Cam -->|Point Cloud| Map[SLAM Node: RTAB-Map]
    
    CV -->|Bounding Boxes & Coordinates| Plan[Path Planning Node]
    Map -->|Occupancy Grid / Pose| Plan
    
    Plan -->|Twist Commands| Base[Hardware/Motor Control]
    
    %% Styling
    classDef rosNode fill:#1168bd,stroke:#0b4c8c,stroke-width:2px,color:#fff;
    classDef hardware fill:#444,stroke:#333,stroke-width:2px,color:#fff;
    
    class CV,Map,Plan rosNode;
    class Cam,Base hardware;
