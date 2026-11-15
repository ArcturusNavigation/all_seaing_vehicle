#!/usr/bin/env python3
"""
Docking task server skeleton (RoboBoat 2027, Task 3 "Navigate the Marina", Core tier).

Task: enter the marina and dock in the slip whose color indicator is GREEN (an available
bay). Slips with a RED indicator, or with a boat already in them, are not available.

This file keeps the plumbing of roboboat/docking.py (subscriptions, parameters, label
loading, PID objects, markers) and leaves the task logic to you. Search for "TODO".
The working 2026 implementation is roboboat/docking.py. Try your own design first and
use that file to compare afterwards.

Specification
  Subscribes:
    object_planes/global (LabeledObjectPlaneArray): banner planes from the RANSAC detector
        after SLAM, in the global frame (`global_frame_id`, default "map"). Each plane has
        a label, a center, and a normal that points out of the slip toward open water.
  Publishes:
    control_options (ControlOption, through send_vel_cmd): body-frame velocity commands,
        x forward and y left in m/s, yaw rate in rad/s.
    docking_marker_pub (MarkerArray): debug markers.
  Actions served:
    docking (Task), search_docking (Search). These are the same names docking.py uses, so
    run_tasks.py works unchanged. Launch only one of docking.py and this file at a time.
  Parameters: declared in __init__, names match docking.py, so the existing docking
    parameter blocks in sim.launch.py and tasks.launch.py apply to this node as they are.
    Parameters in those blocks that this file does not declare are ignored.

How to run it: this file is not installed yet. Add it to install(PROGRAMS ...) in
all_seaing_autonomy/CMakeLists.txt, make it executable (chmod +x), rebuild, and set the
`executable` of the docking node in sim.launch.py to "docking_template.py".
"""
import os
from dataclasses import dataclass
from enum import Enum

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

import all_seaing_common.report_pb2
from all_seaing_common.task_server_base import TaskServerBase
from all_seaing_controller.pid_controller import CircularPID, PIDController
from all_seaing_interfaces.msg import LabeledObjectPlane, LabeledObjectPlaneArray

# Helpers you will probably want:
#   all_seaing_autonomy/geometry_utils.py: angle_between(v1, v2) -> signed angle from v1 to v2
#   all_seaing_autonomy/roboboat/visualization_tools.py: VisualizationTools.visualize_line(...)
#   all_seaing_controller/potential_field.py: PotentialField (LiDAR obstacle avoidance)


@dataclass
class DockSlot:
    """One docking slip, described by the banner at its back."""
    label: int          # banner label (see shape_label_mappings.yaml / all_label_mappings.yaml)
    ctr: np.ndarray     # banner center [x, y] in the global frame, meters
    normal: np.ndarray  # unit vector [x, y] pointing out of the slip, toward open water


# A suggested state machine. Change it if your design needs different states.
# TaskServerBase logs every change of self.state to the run log.
class DockingState(Enum):
    WAITING_DOCK = 1     # no available slip selected yet
    NAVIGATING_DOCK = 2  # driving to a point in front of the selected slip (nav server)
    DOCKING = 3          # final approach into the slip (direct velocity commands)
    UNDOCKING = 4        # backing out of the slip


# TaskServerBase turns this node into a ROS 2 action server and calls these hooks:
#   1. should_accept_task() once, when run_tasks.py asks us to start
#   2. init_setup() repeatedly, until it calls mark_successful()
#   3. control_loop() repeatedly (every timer_period, 30 Hz by default), until it calls
#      mark_successful() or mark_unsuccessful()
# Read all_seaing_common/task_server_base.py to see the helpers you inherit.
class Docking(TaskServerBase):
    def __init__(self):
        super().__init__(
            server_name="docking_server",
            action_name="docking",
            search_action_name="search_docking",
        )

        # Banner planes in the global frame. plane_cb stores the latest message.
        self.plane_msg = None
        self.obj_plane_sub = self.create_subscription(
            LabeledObjectPlaneArray, "object_planes/global", self.plane_cb, qos_profile_sensor_data
        )

        self.docking_marker_pub = self.create_publisher(MarkerArray, "docking_marker_pub", 10)

        # Slip geometry (meters). The sim dock is larger than the real one, so these are
        # set per launch file.
        self.dock_width = self.declare_and_get("dock_width", 1.0)
        self.dock_length = self.declare_and_get("dock_length", 3.0)

        # A boat banner closer than this to a slip banner (meters) may mean the slip is taken.
        self.boat_dock_dist_thres = self.declare_and_get("boat_dock_dist_thres", 5.0)

        # Distance in front of the banner (meters, along the slip normal) for the approach waypoint.
        self.wpt_banner_dist = self.declare_and_get("wpt_banner_dist", 4.0)

        # Switch from the nav server to the final approach when closer than this (meters).
        self.navigation_dist_thres = self.declare_and_get("navigation_dist_thres", 7.0)

        # Two detections closer than this (meters) are treated as the same banner.
        self.duplicate_dist = self.declare_and_get("duplicate_dist", 0.5)

        # Position tolerance (meters) for "the boat is docked".
        self.docked_xy_thres = self.declare_and_get("docked_xy_thres", 0.2)

        # PID gains [Kp, Ki, Kd] and velocity limits [x m/s, y m/s, yaw rad/s] for the final approach.
        Kpid_x = self.declare_and_get("Kpid_x", [0.75, 0.0, 0.0])
        Kpid_y = self.declare_and_get("Kpid_y", [0.75, 0.0, 0.0])
        Kpid_theta = self.declare_and_get("Kpid_theta", [0.75, 0.0, 0.0])
        self.max_vel = self.declare_and_get("max_vel", [2.0, 1.0, 0.4])

        # PIDController: set_setpoint(v), update(measurement, dt), get_effort(), reset().
        # CircularPID does the same for angles and handles the wrap at +-pi.
        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()

        self.load_labels()

        self.state = DockingState.WAITING_DOCK
        self.selected_slot = None  # DockSlot, or None when no available slip is known
        self.reported_docking = False

    def declare_and_get(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def load_labels(self):
        '''
        Fills the label lists used to tell banners apart:
          self.dock_green_labels: slip banners with a green indicator (available)
          self.dock_red_labels:   slip banners with a red indicator (not available)
          self.boat_labels:       banners on boats that may occupy a slip
        Check membership with: plane.label in self.dock_green_labels
        '''
        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        mappings_file = self.declare_and_get(
            "shape_label_mappings_file",
            os.path.join(bringup_prefix, "config", "perception", "shape_label_mappings.yaml"),
        )
        with open(mappings_file, "r") as f:
            self.label_mappings = yaml.safe_load(f)
        # label id -> name, for readable log messages
        self.inv_label_mappings = {value: key for key, value in self.label_mappings.items()}

        self.boat_labels = [self.label_mappings[name] for name in ["black_cross", "black_triangle"]]

        if self.is_sim:
            # The simulated dock has shape banners and no color indicators, so every
            # slip banner counts as available (same as docking.py).
            self.dock_green_labels = [self.label_mappings[name] for name in [
                "blue_circle", "blue_cross", "blue_triangle",
                "green_circle", "green_cross", "green_square", "green_triangle",
                "red_circle", "red_cross", "red_triangle", "red_square",
            ]]
            self.dock_red_labels = []
        else:
            # TODO (real boat): fill these from the label names of the 2027 YOLO model once
            # it is trained. Do not copy the 2026 number_* labels without checking that the
            # 2027 dock still uses them.
            self.dock_green_labels = []
            self.dock_red_labels = []
            self.get_logger().warn("docking_template: no dock labels configured for the real boat yet")

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    def plane_cb(self, msg: LabeledObjectPlaneArray):
        self.plane_msg = msg
        if self.paused:  # True while this task is not running
            return
        self.find_docking_slot()

    def ctr_normal(self, plane: LabeledObjectPlane):
        '''
        Returns (center, normal) of a banner plane as np.array([x, y]) each, in the global
        frame. The normal is a unit vector that points away from the banner, toward the side
        the banner was observed from (out of the slip).
        '''
        center = np.array([plane.normal_ctr.position.x, plane.normal_ctr.position.y])
        q = plane.normal_ctr.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return center, np.array([np.cos(theta), np.sin(theta)])

    def find_docking_slot(self):
        '''
        Reads self.plane_msg and updates self.selected_slot.
        Returns True if an available slip is selected, False otherwise.
        '''
        if self.plane_msg is None:
            return False

        # TODO (step 1): choose a slip.
        #   - Loop over self.plane_msg.objects. Use ctr_normal() for the geometry and the
        #     label lists from load_labels() to tell slip banners from boat banners.
        #   - Decide which slips are available. Things to think about:
        #       * a red indicator means not available
        #       * a boat banner near a slip banner may mean a boat is in that slip
        #         (self.boat_dock_dist_thres); how do you tell "in the slip" from "next to it"?
        #       * detections flicker and labels can be wrong for a few frames; should one
        #         bad frame change your decision?
        #   - If several slips are available, decide which one to take, and when it is worth
        #     switching after you have started driving toward one.
        #   - The same slip is re-detected at slightly different positions every message.
        #     Use self.duplicate_dist to recognise it and refresh self.selected_slot.
        #   - Record decisions in the run log so they can be analysed after a run, e.g.
        #       self.log_event("INFO", "dock_slot_selected", label=..., ctr=[...])
        #       self.log_event("INFO", "dock_slot_rejected", label=..., reason="red indicator")
        return self.selected_slot is not None

    # ------------------------------------------------------------------
    # TaskServerBase hooks
    # ------------------------------------------------------------------

    def should_accept_task(self, goal_request):
        # Start only if an available slip is already visible. The Search action of the base
        # class also calls this (with goal_request=None) while driving to the task location.
        self.selected_slot = None
        self.state = DockingState.WAITING_DOCK
        return self.find_docking_slot()

    def init_setup(self):
        # Reset per-run state, since the same node can run the task more than once.
        self.x_pid.reset()
        self.y_pid.reset()
        self.theta_pid.reset()
        self.reported_docking = False
        self.mark_successful()

    def control_loop(self):
        # Called at 30 Hz. Keep each call short: compute one command and return. The base
        # class handles cancellation between calls.
        if self.state == DockingState.WAITING_DOCK:
            self.wait_for_slot()
        elif self.state == DockingState.NAVIGATING_DOCK:
            self.navigate_to_slot()
        elif self.state == DockingState.DOCKING:
            self.dock()
        elif self.state == DockingState.UNDOCKING:
            self.undock()
        self.publish_markers()

    # ------------------------------------------------------------------
    # Task steps: one method per state
    # ------------------------------------------------------------------

    def wait_for_slot(self):
        '''
        No slip selected. Decide what the boat does until find_docking_slot() selects one.
        '''
        # TODO (step 2): move to NAVIGATING_DOCK once self.selected_slot is set.
        #   Optional: hold position or turn slowly to look for the dock instead of drifting
        #   (self.send_vel_cmd(x, y, theta)).
        self.get_logger().warn("wait_for_slot() is not implemented yet", throttle_duration_sec=5.0)

    def navigate_to_slot(self):
        '''
        Drive to a point in front of the selected slip, using the navigation server so the
        boat plans around the dock and other obstacles.
        '''
        # TODO (step 3):
        #   - Compute a waypoint in front of the slip from self.selected_slot.ctr,
        #     self.selected_slot.normal and self.wpt_banner_dist.
        #   - Send it with self.send_waypoint_to_server(waypoint). That call does not block.
        #     Send it once, then again only when the waypoint has moved noticeably
        #     (self.sent_waypoint holds the last one), or when self.waypoint_rejected /
        #     self.waypoint_aborted is set.
        #   - If the selected slip stops being available on the way, cancel with
        #     self.cancel_navigation() and go back to WAITING_DOCK.
        #   - When the boat is within self.navigation_dist_thres of the slip, call
        #     self.cancel_navigation() and switch to DOCKING.
        self.get_logger().warn("navigate_to_slot() is not implemented yet", throttle_duration_sec=5.0)
        self.mark_unsuccessful()

    def dock(self):
        '''
        Final approach: drive into the slip with direct velocity commands.
        '''
        # TODO (step 4):
        #   - Work in the frame of the slip. From self.robot_pos, self.robot_dir and the
        #     selected slot, compute:
        #       * the sideways offset from the slip centerline,
        #       * the remaining distance along the slip (account for self.dock_length and
        #         the length of the boat so the bow stops before the banner),
        #       * the heading error relative to the slip direction (angle_between helps).
        #   - Feed these to self.x_pid / self.y_pid / self.theta_pid with update_pid(), read
        #     the efforts, and remember that send_vel_cmd() takes velocities in the BOAT
        #     frame while your errors are in the SLIP frame.
        #   - Limit the command with scale_thrust(), then self.send_vel_cmd(x, y, theta).
        #   - The slip walls are obstacles. Decide whether you need the LiDAR potential
        #     field here (see how docking.py uses PotentialField) or whether staying on the
        #     centerline is enough.
        #   - Report the slip to the judges once (see report_docking()).
        #   - When both position errors are below self.docked_xy_thres, switch to UNDOCKING.
        self.get_logger().warn("dock() is not implemented yet", throttle_duration_sec=5.0)
        self.mark_unsuccessful()

    def undock(self):
        '''
        Back out of the slip, stop, and finish the task.
        '''
        # TODO (step 5):
        #   - Back out until the boat is clear of the slip (by distance or by time).
        #   - Send a zero velocity command, then call self.mark_successful().
        self.get_logger().warn("undock() is not implemented yet", throttle_duration_sec=5.0)
        self.mark_unsuccessful()

    # TODO (Advanced tier, after Core works): after docking, "put out a fire" shown by a RED
    # light by spraying water at it. This needs a detection of the light and the water
    # mechanism; see roboboat/mechanism_navigation.py and roboboat/delivery_server.py for
    # how the 2026 delivery task aimed and fired.

    # ------------------------------------------------------------------
    # Control helpers
    # ------------------------------------------------------------------

    def update_pid(self, x, y, heading):
        '''
        Feeds one measurement to the three PIDs. x and y in meters, heading in radians.
        The setpoints default to 0, so pass errors (or set setpoints with set_setpoint()).
        '''
        now = self.get_clock().now()
        dt = (now - self.prev_update_time).nanoseconds / 1e9
        self.x_pid.update(x, dt)
        self.y_pid.update(y, dt)
        self.theta_pid.update(heading, dt)
        self.prev_update_time = now

    def scale_thrust(self, x_vel, y_vel):
        '''
        Scales (x_vel, y_vel) down together so neither exceeds self.max_vel, keeping the
        direction of travel.
        '''
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel
        scale = min(
            self.max_vel[0] / abs(x_vel) if x_vel != 0 else float("inf"),
            self.max_vel[1] / abs(y_vel) if y_vel != 0 else float("inf"),
        )
        return scale * x_vel, scale * y_vel

    def report_docking(self, dock, slip):
        '''
        Reports the chosen slip to the competition server, once per task run.
        dock: dock identifier string, slip: slip identifier string. Check the 2027
        communications protocol for the allowed values before the competition.
        '''
        if self.reported_docking:
            return
        self.report_data(all_seaing_common.report_pb2.Docking(dock=dock, slip=slip))
        self.reported_docking = True

    def publish_markers(self):
        # Clears old markers. TODO: add markers for the selected slip, the approach waypoint
        # and the commanded velocity. Seeing them in Foxglove/RViz saves a lot of debugging.
        marker_arr = MarkerArray(markers=[Marker(id=0, action=Marker.DELETEALL)])
        self.docking_marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    node = Docking()
    # Two threads: one runs the task, the other keeps receiving plane messages.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
