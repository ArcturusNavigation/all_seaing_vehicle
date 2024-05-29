from all_seaing_vehicle.competitions.roboboat.Task import Task
from math import acos, pi
from scipy.spatial.transform import Rotation as R

same_color_midpoint_dist = 40
same_color_max_dist = 80
same_color_min_dist = 30

diff_color_midpoint_dist = 20
diff_color_max_dist = 30
diff_color_min_dist = 10

same_color_dist_weight = 1/5
diff_color_dist_weight = 1/5
turn_angle_weight = 6/pi
cross_angle_weight = 6/pi
outgoing_angle_weight = 0.0

same_color_max = (same_color_max_dist - same_color_min_dist)/2 * 1.5
diff_color_max = (diff_color_max_dist - diff_color_min_dist)/2 * 1.5
turn_angle_max = 1.5
cross_angle_max = 1.0
outgoing_angle_max = 0.85

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cached_magnitude = None

    def magnitude(self):
        if(self.cached_magnitude is None):
            self.cached_magnitude = (self.x ** 2 + self.y ** 2)**0.5
        return self.cached_magnitude
    
    def dot_product(self, other):
        return self.x * other.x + self.y * other.y
    
    def angle_with(self, other):
        return acos(max(-1, min(1, self.dot_product(other) / (self.magnitude() * other.magnitude()))))
    
    def multiply_by_scalar(self, other):
        assert type(other) is float or type(other) is int
        newvec = Vector(self.x * other, self.y * other)
        if self.cached_magnitude is not None:
            newvec.cached_magnitude = self.cached_magnitude / other
        return newvec
    
    def projection_onto(self, other):
        return self.dot_product(other) / other.magnitude()
    
    def __mul__(self, other):
        return self.multiply_by_scalar(other)
    
    def __truediv__(self, other):
        assert other != 0
        return self.multiply_by_scalar(1/other)
    
    def __add__(self, other):
        assert type(other) is Vector
        return Vector(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        assert type(other) is Vector
        return Vector(self.x - other.x, self.y - other.y)

    def __str__(self):
        return f"({self.x}, {self.y})"

def get_ordering_item(buoy_list, ordering, index, is_red, stepback):
    return Vector(diff_color_midpoint_dist/2 * (1 if is_red else -1), 1 - stepback) if index < 0 else buoy_list[ordering[index]].position

def get_reference(buoy_list, ordering, is_red, stepback = 1):
    return get_ordering_item(buoy_list, ordering, len(ordering) - stepback, is_red, stepback)

def get_normal(buoy_list, red_index, green_index):
    return Vector(buoy_list[green_index].position.y - buoy_list[red_index].position.y, buoy_list[red_index].position.x - buoy_list[green_index].position.x)

def get_weight_from_pair(buoy_list, green_index, red_index, green_ordering, red_ordering, red_ref, green_ref):
    between_buoys_diff = buoy_list[red_index].position - buoy_list[green_index].position
    diff_color_dist_unweighted = abs(between_buoys_diff.magnitude() - diff_color_midpoint_dist)
    if diff_color_dist_unweighted > diff_color_max:
        return None
    
    reference_buoy_to_buoy = red_ref - green_ref
    cross_angle_unweighted = reference_buoy_to_buoy.angle_with(between_buoys_diff)
    if cross_angle_unweighted > cross_angle_max:
        return None
    
    vector_to_point = (buoy_list[red_index].position + buoy_list[green_index].position - green_ref - red_ref)/2
    green_secondary_ref = get_reference(buoy_list, green_ordering, False, 2)
    red_secondary_ref = get_reference(buoy_list, red_ordering, True, 2)
    previous_vector = (green_ref + red_ref - green_secondary_ref - red_secondary_ref)/2
    turn_angle_unweighted = previous_vector.angle_with(vector_to_point)
    if turn_angle_unweighted > turn_angle_max:
        return None

    consider_angle = True
    
    if not green_ordering:
        projection = vector_to_point.projection_onto(between_buoys_diff)
        if abs(projection / between_buoys_diff.magnitude()) < 0.5:
            consider_angle = False

    outgoing_angle_unweighted = 0

    if consider_angle:
        outgoing_angle_unweighted = get_normal(buoy_list, red_index, green_index).angle_with(vector_to_point)

    if outgoing_angle_unweighted > outgoing_angle_max:
        return None
    
    return (diff_color_dist_unweighted * diff_color_dist_weight + 
            cross_angle_unweighted * cross_angle_weight + 
            turn_angle_unweighted * turn_angle_weight + 
            outgoing_angle_unweighted * outgoing_angle_weight)

def get_individual_weight(buoy_list, index, ordering, ref):
    diff = buoy_list[index].position - ref
    distance_feet = abs(diff.magnitude() - (same_color_midpoint_dist if ordering else 0))
    if not ordering:
        distance_feet -= same_color_midpoint_dist
    if distance_feet > same_color_max:
        return None
    return distance_feet * same_color_dist_weight

class WeightResult:
    def __init__(self, weight, green_index, red_index):
        self.weight = weight
        self.green_index = green_index
        self.red_index = red_index

def choose_next_pair(buoy_list, green_set, red_set, green_ordering, red_ordering):
    green_ref = get_reference(buoy_list, green_ordering, False)
    red_ref = get_reference(buoy_list, red_ordering, True)

    min_weight = None
    min_index = -1
    min_r_index = -1

    red_cache = {}

    for index in green_set: # green loop
        individual_weight = get_individual_weight(buoy_list, index, green_ordering, green_ref)
        if individual_weight is not None:
            for r_index in red_set:
                if r_index not in red_cache:
                    red_cache[r_index] = get_individual_weight(buoy_list, index, red_ordering, red_ref)
                r_individual_weight = red_cache[r_index]
                if r_individual_weight is not None:
                    multicolor_weight = get_weight_from_pair(buoy_list, index, r_index, green_ordering, red_ordering, red_ref, green_ref)
                    if multicolor_weight is not None:
                        total_weight = individual_weight + r_individual_weight + multicolor_weight
                        if min_weight is None or total_weight < total_weight:
                            min_weight = total_weight
                            min_index = index
                            min_r_index = r_index
    return WeightResult(min_weight, min_index, min_r_index)

def generate_path(_buoy_list):
    waypoints = []
    green_ordering = []
    red_ordering = []
    green_buoys = set()
    red_buoys = set()
    buoy_list = []
    for cluster in _buoy_list:
        if cluster.label == 2:
            red_buoys.add(len(buoy_list))
        elif cluster.label == 1:
            green_buoys.add(len(buoy_list))
        buoy_list.append(Buoy(Vector(cluster.avg_point.x, cluster.avg_point.y), cluster.label == 2))
    print([buoy.__str__() for buoy in buoy_list])
    while True:
        pair = choose_next_pair(buoy_list, green_buoys, red_buoys, green_ordering, red_ordering)
        if pair.weight is None:
            break
        green_buoys.remove(pair.green_index)
        red_buoys.remove(pair.red_index)
        green_ordering.append(pair.green_index)
        red_ordering.append(pair.red_index)
        print('chose green', buoy_list[pair.green_index], 'red', buoy_list[pair.red_index])
        waypoints.append((buoy_list[pair.green_index].position + buoy_list[pair.red_index].position)/2)
    return waypoints

class Buoy:
    def __init__(self, position, is_red):
        self.position = position
        self.is_red = is_red

    def __str__(self):
        return f"({'R' if self.is_red else 'G'}, {self.position})"

class FollowThePath(Task):
    def __init__(self, logger):
        self.logger = logger
        self.current_point = None
        self.theta = None
        self.buoy_list = []
    def get_name(self):
        return "Follow The Path"
    def start(self):
        self.current_point = None
        self.theta = None
        self.buoy_list = []
    def update(self):
        print([buoy.label for buoy in self.buoy_list])
        print([vec.__str__() for vec in generate_path(self.buoy_list)])
    def check_finished(self):
        return False
    def end(self):
        pass
    def get_next(self):
        return False
    def receive_odometry(self, msg):
        self.current_point = msg.pose.pose.position
        self.theta = R.from_quat([ # convert between quaternion and yaw value for theta
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        ]).as_euler('xyz')[2] 
    def receive_buoys(self, msg):
        self.buoy_list = msg.clusters