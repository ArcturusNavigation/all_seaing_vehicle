# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 autonomy stack for an Autonomous Surface Vehicle (ASV) developed by the MIT Arcturus team for the RoboBoat competition. The vehicle uses LiDAR, stereo cameras, GPS, and IMU for perception and localization, and executes competition tasks (gate navigation, buoy following, docking, speed challenges, delivery) autonomously.

## Build Commands

This is a ROS 2 workspace. The repo lives at `dev_ws/src/all_seaing_vehicle` inside a Nix-based development environment (via `arcturus_nix`).

```bash
# Build all packages (from dev_ws/)
colcon build

# Build a single package
colcon build --packages-select all_seaing_perception

# Build a package and its dependencies
colcon build --packages-up-to all_seaing_autonomy

# Source the workspace after building
source install/setup.bash

# Run a specific node
ros2 run all_seaing_perception yolov8_node.py

# Launch the full vehicle stack
ros2 launch all_seaing_bringup vehicle.launch.py

# Launch simulation
ros2 launch all_seaing_bringup sim.launch.py

# Launch perception pipeline only
ros2 launch all_seaing_bringup perception.launch.py

# Launch competition tasks
ros2 launch all_seaing_bringup tasks.launch.py
```

## CI

GitHub Actions (`.github/workflows/build.yml`) builds via Nix flake. Perception and bringup packages are excluded from CI due to Nix incompatibility. All other 9 packages are built and their nodes are verified to launch.

## Architecture

### Package Dependency Hierarchy

```
all_seaing_interfaces  (custom msgs/srvs/actions - no code deps)
        ↓
all_seaing_common      (base classes: ActionServerBase, TaskServerBase, QoS, protobuf reporting)
        ↓
┌───────┼───────────┬──────────────┬──────────────┐
driver  perception  navigation    controller     autonomy
```

`all_seaing_bringup` depends on all packages (launch files + configs). `all_seaing_description` is standalone (URDF/xacro only).

### Key Design Patterns

**Task execution model:** Competition tasks are ROS 2 action servers inheriting from `TaskServerBase` → `ActionServerBase`. `TaskServerBase` provides `init_setup()` and `control_loop()` hooks — subclasses override these. The `run_tasks.py` state machine orchestrates task order via action clients.

**Navigation pipeline:** Task servers call `move_to_point()` or `move_to_waypoint()` which sends `FollowPath` or `Waypoint` action goals. The navigation server plans paths (A* or tangent-based), and `controller_server.py` executes them using PID + potential field obstacle avoidance. `control_mux.py` arbitrates between autonomous and teleop commands.

**Perception pipeline:** Camera images → YOLO detection (`yolov8_node.py` / `yolov11_*.py`) + color segmentation → `bbox_project_pcloud` (C++) fuses 2D detections with LiDAR point clouds → `object_tracking_map` (C++) tracks obstacles over time using Hungarian algorithm → publishes `ObstacleMap` messages consumed by navigation and task servers.

### Build System Pattern

All packages use `ament_cmake_auto` (not pure `ament_python`). Python nodes are installed as executable scripts via `install(PROGRAMS ...)` in CMakeLists.txt, and Python libraries via `ament_python_install_package()`. C++ executables use `ament_auto_add_executable()`. New Python nodes must be added to both the filesystem and the `install(PROGRAMS ...)` block in the package's CMakeLists.txt.

### Custom Interfaces (all_seaing_interfaces)

- **Actions:** `FollowPath` (path planning + following), `Waypoint` (direct point navigation), `Task` (competition task execution), `Search` (find task targets)
- **Key messages:** `Obstacle` / `ObstacleMap` (tracked 3D obstacles with labels, hulls, bounding boxes), `LabeledBoundingBox2D` / `Array` (2D detections), `ControlOption` (velocity commands with priority)
- **Services:** `PlanPath` (request path from planner), `CommandAdj`/`CommandServo`/`CommandFan` (hardware actuators), `RestartSLAM`

### C++ Components (Performance-Critical)

Located in `all_seaing_perception/` and `all_seaing_navigation/`: obstacle detection, point cloud filtering, RANSAC plane detection, object tracking (Hungarian algorithm), A* pathfinding. Linked against PCL 1.12, Eigen3, yaml-cpp.

### Configuration

All YAML configs live in `all_seaing_bringup/config/` organized by subsystem (`localization/`, `slam/`, `perception/`). Device-specific configs are in `all_seaing_driver/config/`. Thruster and robot model configs are in `all_seaing_description/config/` and `urdf/`.

### Launch Files

Main launch files in `all_seaing_bringup/launch/`: `vehicle.launch.py` (real robot), `sim.launch.py` (Gazebo simulation), `perception.launch.py`, `tasks.launch.py`. These compose nodes with parameters from the config YAML files. Driver-specific launches are in `all_seaing_driver/launch/`.
