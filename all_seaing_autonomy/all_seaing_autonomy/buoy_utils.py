import math
import numpy as np
from all_seaing_interfaces.msg import Obstacle
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler
from all_seaing_autonomy.geometry_utils import ccw


class InternalBuoyPair:
    def __init__(self, left_buoy=None, right_buoy=None):
        if left_buoy is None:
            self.left = Obstacle()
        else:
            self.left = left_buoy

        if right_buoy is None:
            self.right = Obstacle()
        else:
            self.right = right_buoy


def ob_coords(buoy, local=False):
    if local:
        return np.array([buoy.local_point.point.x, buoy.local_point.point.y])
    else:
        return np.array([buoy.global_point.point.x, buoy.global_point.point.y])


def get_closest_to(source, buoys, local=False):
    return min(
        buoys,
        key=lambda buoy: np.linalg.norm(source - ob_coords(buoy, local)),
    )


def midpoint_pair_dir(pair, forward_dist):
    left_coords = ob_coords(pair.left)
    right_coords = ob_coords(pair.right)
    mid = (left_coords + right_coords) / 2
    diff = right_coords - left_coords
    n = np.linalg.norm(diff)
    if n == 0:
        # If buoys are at the same position, return mid with no direction
        return mid, np.array([1.0, 0.0])  # arbitrary direction
    diff = diff / n
    perp = np.array([-diff[1], diff[0]])
    mid = mid + forward_dist * perp
    return mid, perp


def split_buoys(obstacles, green_labels, red_labels):
    green_bouy_points = []
    red_bouy_points = []
    for obstacle in obstacles:
        if obstacle.label in green_labels:
            green_bouy_points.append(obstacle)
        elif obstacle.label in red_labels:
            red_bouy_points.append(obstacle)
    return green_bouy_points, red_bouy_points


def obs_to_pos(obs):
    return [ob_coords(ob, local=False) for ob in obs]


def obs_to_pos_label(obs):
    return [(*ob_coords(ob, local=False), ob.label) for ob in obs]


def filter_front_buoys(pair, buoys, buoy_pair_dist_thres):
    return [
        buoy
        for buoy in buoys
        if (ccw(
            ob_coords(pair.left),
            ob_coords(pair.right),
            ob_coords(buoy),
        ) and (min(np.linalg.norm(ob_coords(buoy) - ob_coords(pair.left)),
                   np.linalg.norm(ob_coords(buoy) - ob_coords(pair.right))) > buoy_pair_dist_thres))
    ]


def pick_buoy(buoys, prev_mid, ref_buoy, duplicate_dist):
    buoys.sort(key=lambda buoy: np.linalg.norm(prev_mid - ob_coords(buoy)))
    for buoy in buoys:
        if np.linalg.norm(ob_coords(ref_buoy) - ob_coords(buoy)) > duplicate_dist:
            return False, buoy
    return True, ref_buoy


def replace_closest(ref_obs, obstacles, duplicate_dist):
    if len(obstacles) == 0:
        return ref_obs, False
    opt_buoy = get_closest_to(ob_coords(ref_obs), obstacles)
    if np.linalg.norm(ob_coords(ref_obs) - ob_coords(opt_buoy)) < duplicate_dist:
        return opt_buoy, True
    else:
        return ref_obs, False


def buoy_pairs_distance(p1, p2, mode="min", local=False):
    p1_left = ob_coords(p1.left, local=local)
    p1_right = ob_coords(p1.right, local=local)
    p2_left = ob_coords(p2.left, local=local)
    p2_right = ob_coords(p2.right, local=local)
    if mode == "mid":
        p1_mid = (p1_left + p1_right) / 2
        p2_mid = (p2_left + p2_right) / 2
        dist = np.linalg.norm(p1_mid - p2_mid)
    else:
        dist = min(np.linalg.norm(p2_left - p1_left), np.linalg.norm(p2_right - p1_right))
    return dist


def buoy_pairs_angle(p1, p2, local=False):
    p1_left = ob_coords(p1.left, local=local)
    p1_right = ob_coords(p1.right, local=local)
    p2_left = ob_coords(p2.left, local=local)
    p2_right = ob_coords(p2.right, local=local)
    p1_diff = p1_right - p1_left
    p2_diff = p2_right - p2_left
    denom = np.linalg.norm(p1_diff) * np.linalg.norm(p2_diff)
    if denom == 0:
        return 0.0
    angle = math.acos(np.clip((p1_diff @ p2_diff) / denom, -1.0, 1.0))
    return angle


def get_acute_angle(angle):
    ret_angle = angle
    if ret_angle < 0:
        ret_angle = -ret_angle
    if ret_angle > math.pi / 2.0:
        ret_angle = math.pi - ret_angle
    return ret_angle


def get_triangle_angle(buoy_a, buoy_b, buoy_c):
    return get_acute_angle(buoy_pairs_angle(InternalBuoyPair(buoy_a, buoy_b), InternalBuoyPair(buoy_b, buoy_c)))


def check_better_pair_angles(ref_pair, old_pair, new_pair, better_angle_thres, duplicate_dist, mode="both"):
    old_left_duplicate = (np.linalg.norm(ob_coords(ref_pair.left) - ob_coords(old_pair.left)) < duplicate_dist)
    old_right_duplicate = (np.linalg.norm(ob_coords(ref_pair.right) - ob_coords(old_pair.right)) < duplicate_dist)
    new_left_duplicate = (np.linalg.norm(ob_coords(ref_pair.left) - ob_coords(new_pair.left)) < duplicate_dist)
    new_right_duplicate = (np.linalg.norm(ob_coords(ref_pair.right) - ob_coords(new_pair.right)) < duplicate_dist)

    old_left = 0 if old_left_duplicate else get_triangle_angle(ref_pair.left, old_pair.left, old_pair.right)
    old_right = 0 if old_right_duplicate else get_triangle_angle(ref_pair.right, old_pair.right, old_pair.left)
    new_left = 0 if new_left_duplicate else get_triangle_angle(ref_pair.left, new_pair.left, new_pair.right)
    new_right = 0 if new_right_duplicate else get_triangle_angle(ref_pair.right, new_pair.right, new_pair.left)

    if ((old_left_duplicate or old_right_duplicate) and (new_left_duplicate or new_right_duplicate)):
        old_angle = old_left if old_right_duplicate else old_right
        new_angle = new_left if new_right_duplicate else new_right
        return new_angle > old_angle + better_angle_thres
    elif ((old_left_duplicate or old_right_duplicate) and (new_left_duplicate or new_right_duplicate)):
        return (old_left_duplicate or old_right_duplicate)
    else:
        if mode == "both":
            return (new_left > (old_left + better_angle_thres)) and (new_right > (old_right + better_angle_thres))
        else:
            return (new_left > (old_left + better_angle_thres)) or (new_right > (old_right + better_angle_thres))


def better_buoy_pair_transition(p_old, p_new, p_ref, buoy_pair_dist_thres, better_angle_thres, duplicate_dist, mode="both"):
    return (buoy_pairs_distance(p_ref, p_new, "mid") > buoy_pair_dist_thres and
            (buoy_pairs_distance(p_ref, p_old, "mid") <= buoy_pair_dist_thres or
             check_better_pair_angles(p_ref, p_old, p_new, better_angle_thres, duplicate_dist, mode)))


def check_better_one_side(ref_buoy, old_buoy, new_buoy, better_angle_thres):
    old_angle = get_triangle_angle(ref_buoy, old_buoy, new_buoy)
    new_angle = get_triangle_angle(ref_buoy, new_buoy, old_buoy)
    return new_angle > (old_angle + better_angle_thres)

def buoy_pairs_to_markers(buoy_pairs, red_left, global_frame_id="map"):
        """
        Create the markers from an array of buoy pairs to visualize them (and the respective waypoints) in RViz
        """
        marker_array = MarkerArray()
        i = 0
        for p_left, p_right, point, radius in buoy_pairs:
            marker_array.markers.append(
                Marker(
                    type=Marker.ARROW,
                    pose=point,
                    header=Header(frame_id=global_frame_id),
                    scale=Vector3(x=2.0, y=0.15, z=0.15),
                    color=ColorRGBA(a=1.0, b=1.0),
                    id=(4 * i),
                )
            )
            if red_left:
                left_color = ColorRGBA(r=1.0, a=1.0)
                right_color = ColorRGBA(g=1.0, a=1.0)
            else:
                left_color = ColorRGBA(g=1.0, a=1.0)
                right_color = ColorRGBA(r=1.0, a=1.0)

            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=pair_to_pose(ob_coords(p_left)),
                    header=Header(frame_id=global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=left_color,
                    id=(4 * i) + 1,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=pair_to_pose(ob_coords(p_right)),
                    header=Header(frame_id=global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=right_color,
                    id=(4 * i) + 2,
                )
            )
            # marker_array.markers.append(
            #     Marker(
            #         type=Marker.CYLINDER,
            #         pose=Pose(
            #             position=Point(
            #                 x=point.position.x,
            #                 y=point.position.y,
            #             )
            #         ),
            #         header=Header(frame_id=self.global_frame_id),
            #         scale=Vector3(
            #             x=radius, y=radius, z=1.0
            #         ),
            #         color=ColorRGBA(g=1.0, a=0.5),
            #         id=(4 * i) + 3,
            #     )
            # )
            i += 1
        return marker_array

def pair_to_pose(pair):
    return Pose(position=Point(x=pair[0], y=pair[1]))

def pair_angle_to_pose(pair, angle):
    quat = quaternion_from_euler(0, 0, angle)
    return Pose(
        position=Point(x=pair[0], y=pair[1]),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    )
