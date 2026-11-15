# all_seaing_vehicle

> **Read and follow [`RULES.md`](RULES.md) first.** In short: no changes unless asked, ask when unclear, keep code clean and reusable and document it (including the wiki).

ROS 2 (Humble, Ubuntu 22.04) autonomy stack for **MIT Arcturus**'s autonomous surface vehicle (ASV), which competes in **RoboBoat**. The current vehicle is *Fish 'N Ships*: a ~4 ft catamaran with four Blue Robotics T200 thrusters in an **X-drive** (holonomic), a Jetson AGX Orin, an HDL-32E Velodyne LiDAR, one ZED 2i plus two OAK-1 W cameras (~300° FOV), a ZED-F9P RTK GPS (base station on shore), a Pixhawk 6X (EKF3, external compass), and a custom CAN-bus electrical system with remote E-stop.

- Team wiki (setup, tutorials, per-package docs): https://github.com/ArcturusNavigation/all_seaing_vehicle/wiki. Some pages are stale (e.g. they still describe `yolov8_node.py` and EKF SLAM as current), so treat the code as the source of truth.
- Design reports and test logs: https://arcturus.mit.edu (TDRs under `/documents/`, testing blog under `/blog/testing`).
- Team autonomy notes (Notion "Autonomy Wiki": planning, Jetson/GPS setup, perception/SLAM design notes, thruster configuration study): https://neon-coal-0ba.notion.site/Autonomy-Wiki-1c6400e9c6ad80ffae90dc9d9b2f4f0f. A summary is under "Operational notes and design context" below.

## Workspace, build, run

Expected layout: `~/arcturus/dev_ws/src/all_seaing_vehicle` (this repo), plus `~/arcturus/vrx_ws/src/vrx` (the team's fork of Virtual RobotX / Gazebo Garden, for simulation). The Docker alternative is `ArcturusNavigation/arcturus_docker`.

```bash
cd ~/arcturus/dev_ws
colcon build --symlink-install [--packages-select <pkg> ...]   # add --executor sequential if OOM
source /opt/ros/humble/setup.bash && source install/setup.bash && source ~/arcturus/vrx_ws/install/setup.bash

ros2 launch all_seaing_bringup sim.launch.py [no_gui:=true] [launch_rviz:=false]   # VRX sim + full stack
```

In sim, the keyboard window toggles teleop/autonomous with Enter and E-stops with Space (WASDQE drive). RViz "Publish Point" sends a goal.

On the boat (SSH to the Jetson), split into launch files so the slow parts can stay up while task code is restarted:

```bash
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB*
ros2 launch all_seaing_bringup vehicle.launch.py location:=<loc> comms:=custom   # drivers, mux, x-drive, central hub, odometry
ros2 run mavros mav sys rate --all 30                                             # MUST run every time (Pixhawk rate resets; slow odom breaks SLAM)
ros2 launch all_seaing_bringup perception.launch.py location:=<loc>              # YOLO, fusion, SLAM, grid map
ros2 launch all_seaing_bringup tasks.launch.py location:=<loc>                   # controller/nav servers, task servers, run_tasks
ros2 launch all_seaing_bringup shoreside.launch.py comms:=...                     # on shore laptop, AFTER vehicle.launch.py
ros2 service call restart_slam all_seaing_interfaces/RestartSLAM "{restart_buoys: true, restart_banners: true, restart_position: true}"
```

`location` values (datum lat/lon/heading, indoor/outdoor) are listed in `all_seaing_bringup/config/localization/locations.yaml`. To replay rosbags, use `tasks_rosbag.launch.py` and `all_seaing_perception/launch/*_rosbag*.launch.py`.

There are **no unit tests**. Validate changes in sim (VRX), then against rosbags, then on the water. CI (`.github/workflows/build.yml`) builds through Nix (`ArcturusNavigation/arcturus_nix`) and skips `all_seaing_perception` and `all_seaing_bringup`.

## Architecture

Four layers, top to bottom. Everything above the drivers talks through ROS actions, so goals can be cancelled or aborted and the caller gets feedback.

1. **Task manager**: `all_seaing_autonomy/roboboat/run_tasks.py`. A state machine over the `Task`/`Search` action clients. The task order is a hard-coded list in `__init__`; entries are commented in or out per test.
2. **Task servers**: `all_seaing_autonomy/roboboat/*.py`. One node per RoboBoat task: `entry_gates`, `follow_buoy_path`, `speed_challenge`, `docking`, `mechanism_navigation`/`delivery_server`, `harbor_alert`, `return_home`, `task_init`. Fallbacks that don't depend on localization use a local map plus PID: `follow_buoy_pid`, `speed_challenge_pid`, `docking_fallback`, `delivery_qual`.
3. **Core subsystems**: navigation, control, perception and SLAM (below).
4. **Drivers**: MAVROS/Pixhawk, central hub (CAN), cameras, LiDAR, GPS, shore comms.

### Control path
`task / nav server` → `waypoint` action (`controller_server.py`: PID on x, y, θ in the robot frame, plus LiDAR potential-field avoidance when `avoid_obs`) → `ControlOption{priority, twist}` on `control_options` → `control_mux.py` (the **lowest** priority value wins: teleop = 0, autonomy = 1; stale options clear every 0.5 s) → `cmd_vel` → `xdrive_controller.py` → `thrusters/{front_right,back_left,front_left,back_right}/thrust` → `thrust_commander.py` (MAVROS) or the VRX thrusters.

`xdrive_controller.py` turns the target body velocity into thruster commands. It solves `min ‖u‖ s.t. B·u = C·v²` with scipy, where `B` comes from the thruster positions and `thruster_angle`, `C` holds the empirical drag constants, and `u ∈ [-1,1]` is scaled to `output_range` (PWM). It also low-pass smooths the output. The derivation is on the wiki page "X-Drive Controller" (TDR 2026, Eq. 11). Thruster order everywhere is **[FR, BL, FL, BR]**.

### Navigation (`all_seaing_navigation`)
- `grid_map_generator.py`: builds an occupancy grid (`/dynamic_map`) from unlabeled LiDAR clusters, using Bayesian occupancy updates.
- `a_star_server.cpp`: the `PlanPath` service (A* that may jump up to 2 cells, then downsamples the path).
- `FollowPath` action servers, selected in the launch files: `navigation_server.py` (A* plus sequential waypoints), `navigation_server_nomap.py`, `navigation_server_tangent.py`, and **`navigation_server_continuous.py` (the current default)**. The continuous server feeds the controller a "carrot" setpoint on the path tangent, publishes `ContinuousWaypoint`/`WaypointStatus`, and replans when the map shifts.

### Perception and SLAM (`all_seaing_perception`, mostly C++)
- `yolov11_all_node.py`: runs YOLOv11 on all three cameras in a single node (TensorRT `.engine` files are in `models/`) and publishes `LabeledBoundingBox2DArray`. `color_segmentation*.py` is the sim/HSV path.
- `point_cloud_filter` (range, intensity and voxel filtering) → `obstacle_detector` (Euclidean clusters → `ObstacleMap`, raw and unlabeled).
- `bbox_project_pcloud` (one instance per camera): projects LiDAR points into the image, clusters the points inside each box, and filters out background, occlusions and reflections by color/cluster matching. It publishes detections in `base_link`. `multicam_detection_merge.py` merges the three cameras into `obstacle_map/local`. `ransac_detector` fits banner planes (center, normal, size).
- SLAM (2D pose plus landmarks; consumes `odometry/integrated` from `all_seaing_driver/odometry_publisher.py`, or RF2O+EKF indoors):
  - **`factor_graph_slam`** (GTSAM, custom `BearingRangePhiFactor`, config `config/slam/slam_factor_graph.yaml`) is the node currently wired as `slam_node` in `sim.launch.py` and `perception.launch.py`.
  - `object_tracking_map` (EKF-SLAM, `slam_real.yaml`) is the older node, still defined in the launch files.
  - Shared data association (greedy, or Hungarian/`linear_sum_assignment`) lives in `object_tracking_shared.cpp`.
  - SLAM outputs `obstacle_map/global`, `object_planes/global`, and the `slam_map`/`map` → `base_link` TF. Tunables are documented on the wiki SLAM page.

### TF frames
Frames follow REP-105 loosely. `odometry_publisher.py` turns GPS (UTM, using the datum from `locations.yaml`) plus Pixhawk velocities and heading into `odom → base_link`. SLAM publishes the corrected global frame. Action servers resolve the robot pose through `ActionServerBase.get_robot_pose()` (a TF lookup from `global_frame_id`, default `map`, to `robot_frame_id`, default `base_link`). Static sensor extrinsics, including the LiDAR–camera calibration, live in `all_seaing_description/launch/static_transforms*.launch.py`.

## Packages
| Package | Contents |
|---|---|
| `all_seaing_interfaces` | msgs (`ObstacleMap`/`Obstacle`, `ControlOption`, `LabeledBoundingBox2D*`, `LabeledObject{PointCloud,Plane}*`, `ContinuousWaypoint`, `WaypointStatus`, `Heartbeat`, …), srvs (`PlanPath`, `RestartSLAM`, `CommandServo/Adj/Fan`, `GetEstopStatus`, …), actions (`Waypoint`, `FollowPath`, `Task`, `Search`) |
| `all_seaing_common` | observability: `RunLogger` (per-run async JSON-lines logs, #501) and `StatusReporter` (live `/diagnostics`, #480), Python + C++ (`all_seaing_observability` library); `ActionServerBase` (TF pose, one-process-at-a-time semaphore with `start_process`/`end_process`/`should_abort`, marker cleanup, protobuf `report_data` for the competition reporting server); `TaskServerBase` (task lifecycle: `should_accept_task` / `init_setup` / `control_loop` / `mark_successful`, waypoint/follow_path clients, buoy-gate helpers, shared task params); `report.proto`/`report_pb2.py`; `qos.py` |
| `all_seaing_autonomy` | task servers (`roboboat/`), `buoy_utils.py` (buoy pairing, splitting red/green, gate poses), `geometry_utils.py` |
| `all_seaing_controller` | `controller_server.py`, `control_mux.py`, `xdrive_controller.py`, `pid_controller.py` (`PIDController`, `CircularPID`), `potential_field.py` |
| `all_seaing_navigation` | see Navigation above; `planner_executor.py`/`planner_base.py`, `rviz_waypoint_sender.py` |
| `all_seaing_perception` | see Perception above; headers in `include/all_seaing_perception/`; training notebooks |
| `all_seaing_driver` | `central_hub_ros.py` + `ArcturusEE.py` (EE boards over serial/CAN: E-stop status, servos, fans), `thrust_commander.py`, `odometry_publisher.py`, `rover_custom_controller.py` / `rover_lora_controller.py` / `onshore_*` (teleop + heartbeat; the boat E-stops if the heartbeat is lost), `sound_signal_detector.py`, `webcam_publisher.py`, `protobuf/` (shore TCP reporting); launch files for mavros, zed2i, oak, Velodyne, u-blox; configs in `config/` |
| `all_seaing_bringup` | top-level launch files (`sim`, `vehicle`, `perception`, `tasks`, `shoreside`, `*_rosbag`, `amcl`, `cartographer`); `config/{perception,localization,slam,course}`; maps and satellite tiles for RViz |
| `all_seaing_description` | URDFs (`xdrive_wamv` is the current one for sim, `simple_wamv`), static transforms |
| `all_seaing_rviz_plugins` | RViz display for `ObstacleMap` |
| `all_seaing_utility` | rosbag tools (`rosbag_decompress_images.py`, `rosbag_shift_time.py`, → image/numpy), `run_log_analysis.py` + `run_log_analyzers/` (per-run log analysis, #501), `topic_monitor.py` (topic rates & stamp ages → `/diagnostics`), `scripts/kill_ros_nodes.sh` (clean staged shutdown + log recovery), TF filtering, tile tools |
| `all_seaing_control_panel` | web app **on the laptop** (`ros2 run all_seaing_control_panel control_panel.py`, http://localhost:8080, #327): run anything from `config/registry.yaml` on the Jetson (SSH) or locally, live `/diagnostics`, pre-run checks (adapt to live / rosbag replay / sim), run-log analysis & plots, parameters (set live, save, write into launch files). Also serves the offline map tiles for Foxglove |
| `all_seaing_foxglove` | not a ROS package (`COLCON_IGNORE`): Foxglove extension (TypeScript) converting our custom messages for Foxglove's panels (replaces `all_seaing_rviz_plugins`), plus Foxglove layouts generated from the RViz configs (`layouts/rviz_to_foxglove.py`) |

## Observability, replay & tools
- Instrument new or changed nodes: `StatusReporter` for live counters/rates/timings, `RunLogger` for per-event decisions with reasons and data. Every path where data is dropped or rejected should record why. See the wiki pages Observability, Control Panel and Foxglove.
- Debug perception and performance on the **Jetson** with `ros2 launch all_seaing_perception perception_raw_rosbag.launch.py bag:=<bag>`. It plays raw sensors only and recomputes everything live; camera topics must be uncompressed (`rosbag_decompress_images.py`), and re-recorded bags need `rosbag_shift_time.py`. Bags live in `/media/arcturus/drive/rosbags/` on the Jetson; run logs in `~/arcturus_logs` (symlink to the NVMe).
- Task-logic logging and observability are tested in **sim**.
- Stop everything with `kill_ros_nodes.sh`, not by killing processes by hand.
- Visualize with Foxglove (`ws://jetson:8765`), not RViz on the laptop, which slows the boat down.

## Conventions
- **Every package is `ament_cmake`, including the Python ones.** A new Python node needs `#!/usr/bin/env python3`, `chmod +x`, and an entry in that package's `CMakeLists.txt` `install(PROGRAMS …)`. Executables keep the `.py` suffix (`ros2 run pkg node.py`). Dependencies go in `package.xml` (the CMake files use `ament_cmake_auto`). Rebuild after adding files or changing C++.
- Use generic topic names inside nodes (`image`, `point_cloud`, `detections`) and remap them in launch files. Parameters are declared with `self.declare_parameter(name, default).get_parameter_value()…`. Tunable values go in `all_seaing_bringup/config/**.yaml` or the launch-file `parameters=`, not hard-coded.
- **Action servers** subclass `ActionServerBase`. Call `start_process()` at the top of the execute callback and `end_process()` on every exit path. Inside loops, poll `should_abort()` and `goal_handle.is_cancel_requested`, and send a zero-velocity `ControlOption` when stopping. Task servers subclass `TaskServerBase` and override its hook methods.
- Executors are `MultiThreadedExecutor`, and loops run on `time.sleep(TIMER_PERIOD)` with `TIMER_PERIOD = 1/60` (controller) or 1/30 (tasks).
- Sim and real behaviour differ through `is_sim` parameters and through which launch file passes which config. Keep both paths working.
- Style: `.clang-format` is LLVM with 4-space indent and a 100-column limit. Python follows the surrounding code (Black/ruff-ish, 4 spaces). Match the existing comment density and naming (`snake_case`; the C++ uses an `m_` member prefix).
- Do not commit machine-specific paths (e.g. a local URDF in `sim.launch.py`), debug logs (`log.txt`, `dmesg.log`), or rosbags.

## Operational notes and design context (from the Notion Autonomy Wiki)

### Jetson and hardware access
- SSH: `ssh arcturus@jetson` (static IP `192.168.8.2` on the `arcturus` router network; the router admin page at `192.168.8.1` lists connected devices). Credentials (router, admin, Jetson, Docker) are in the gitignored local file `CREDENTIALS.local.md` in the repo root, and in Notion/team docs. Never commit them or copy them into tracked files.
- Serial console without the network: connect over micro-USB, find the port with `dmesg | grep tty`, then `sudo screen /dev/ttyACM0 115200`.
- For a remote desktop, use NoMachine. If it shows no desktop or a bad resolution: `sudo systemctl stop gdm && sudo /etc/NX/nxserver --restart`.
- To see which `ttyACM*` is the Pixhawk and which is the central hub: `systemctl | grep ttyACM`. Static device names come from udev rules keyed on vendor/device ID.
- Live visualization: plan to use `foxglove_bridge` instead of RViz on the Jetson (issue #480).

### GPS / RTK
- In practice we usually get DGNSS Fix/Float (corrections received, not enough satellites for RTK Fix). That's fine because we care about local precision, not global accuracy. Galileo HAS is enabled.
- To change the base station's fixed position or switch to survey-in, use u-center: View → Messages → `UBX-CFG-TMODE3`, and save the config there. Configuring through Mission Planner is temporary and has broken the base↔rover radio link before. A fixed position too far from the true one never reaches TIME mode.
- NTRIP/internet corrections (MaCORS) through the Jetson are an open idea, not implemented.

### Replaying rosbags (standard recipe)
Remap away the topics that the nodes under test will republish, play with `--clock`, and launch the matching `*_rosbag` launch file:
```bash
ros2 bag play <bag> --clock [--read-ahead-queue-size 100000] \
  --remap /tf:=/tf_fake /tf_static:=/tf_static_fake /obstacle_map/global:=/obstacle_map/global_fake [<other republished topics>:=..._fake]
ros2 launch all_seaing_perception multicam_slam_rosbag.launch.py     # (or multicam_slam_rosbag_indoors / slam_roboboat_rosbag)
ros2 launch all_seaing_bringup tasks_rosbag.launch.py                # if testing task/nav logic too
```
Notion's Perception page lists the exact remaps for specific test bags (Sept/Oct 2025 outdoor, Jan 2026 indoor, Apr 2026 FGO).

### Localization / SLAM design notes
- Odometry is **pre-integrated in `odometry_publisher.py`** (published as `odometry/integrated`), so SLAM applies the relative motion since its last update. This keeps SLAM consistent even when it processes odometry late. The motion noise is approximated, not exact.
- Pixhawk EKF3 (IMU + compass + GPS) is preferred over `robot_localization` for fused velocities. The compass is the yaw source. The Pixhawk's mounting orientation relative to `base_link` has been wrong before (rotated 90° and flipped between boats): check the `imu`/odom frames.
- Indoors: RF2O LiDAR odometry + EKF (current); AMCL+Cartographer and DLIO were used before.
- The open factor-graph SLAM work (#476) includes: attaching detections and GPS to the pose with the closest timestamp instead of the latest pose, a Huber/robust cost, fixed-lag smoothing (marginalizing old poses and removed landmarks), and batching the Mahalanobis gating.
- The plan for local–global hybrid control (#479): use the local map for immediate reactions when it has enough information. Use the global map to fill gaps (e.g. gate geometry when only one buoy is visible) and to reject fake gates. Global waypoints can be projected into the robot frame for local control.

### Competition reporting
The task manager sends the heartbeat and harbor alert reports. Individual task nodes send their own events (gate passes, objects detected with id/color/position, deliveries, docking slot) through `ActionServerBase.report_data`. The onshore side converts these to the RoboBoat protocol. Label → color/number mappings come from `all_seaing_bringup/config/perception/all_label_mappings.yaml`.

### Simulation
- GPS/IMU noise is configured in the VRX fork: `vrx_urdf/wamv_gazebo/urdf/components/wamv_gps.xacro` (and the IMU equivalent).
- To test a different thruster layout, generate a URDF with `ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=... thruster_yaml:=... wamv_target:=.../wamv_target.urdf wamv_locked:=False`. Then point `sim.launch.py`'s `urdf` argument at it and update the `xdrive_controller.py` thruster positions in `sim.launch.py` to match. Don't commit the local URDF path.
- Thruster configuration study (RoboBoat 2027): the current layout is X-drive (holonomic, but more drag and less forward thrust). Candidates are Azimuth + X-drive (switchable azimuth/X-drive modes) or differential + sway/tunnel thrusters. Pure differential or pure azimuth layouts lose sway control.

## Known pitfalls (from field testing)
- Thruster channel assignment and sign conventions have been flipped by old hotfixes more than once. Verify FR/BL/FL/BR mapping, PID signs, and joystick/x-drive axes in the tank before going on the water.
- The MAVROS stream rate must be 30 Hz (see above). The Pixhawk heading has jumped 90° in the past, so check the odometry/GPS/LiDAR alignment before every run. GPS drifts from multipath near walls; SLAM, or GPS with high uncertainty, compensates.
- At high speed, SLAM and fusion suffer when detections and odometry are out of sync. Keep the perception pipeline fast (YOLO, synchronization, QoS: `best_effort` drops many messages in bags), and close RViz on the Jetson when possible.
- YOLO confusions: red read as yellow under glare, dark water or shade read as green, and the EE box seen by the side cameras. Duplicate or overlapping boxes need filtering before fusion.
- The obstacle map stores convex hulls, so non-convex shapes such as the dock can swallow the robot or the goal and make A* fail.
- Thresholds must agree between layers. For example, the `follow_buoy_path` "passed the gate" check versus the navigation server's `xy_threshold`.
