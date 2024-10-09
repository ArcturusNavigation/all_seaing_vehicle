import heapq

''' These data structures can be used in the search function
First run
ros2 launch path_planning sim_plan.launch.xml
then
ros2 launch racecar_simulator simulate.launch.xml
ssh -L 6081:localhost:6081 racecar@192.168.1.85
'''

class PriorityQueue:
    """
    Priority Queue implementation using heapq
    """
    def __init__(self):
        self.elements = []
        self.element_set = set()

    def empty(self):
        return len(self.elements) == 0

    def put(self, item):
        heapq.heappush(self.elements, item)
        self.element_set.add(item)

    def get(self):
        return heapq.heappop(self.elements)

    def __contains__(self,item):
        return item in self.element_set

class Map():
    """
    Occupancy Grid gives information about the location of obstacles
    grid.data lists the occupancy values of map cells: 100 = occupied, 0 = free, -1 = unknown
    """
    def __init__(self, occupany_grid, lane_traj=None) -> None:
        self._height = occupany_grid.info.height
        self._width = occupany_grid.info.width

        self._resolution = occupany_grid.info.resolution

        p = occupany_grid.info.origin.position #Ros2 Point
        self.origin_p = np.array([[p.x], [p.y], [p.z]]) #3x1

        o = occupany_grid.info.origin.orientation #Ros2 Quaternion
        self.origin_o = [o.x, o.y, o.z, o.w]

        self.R_z = lambda theta: np.array([ [np.cos(theta), -np.sin(theta), 0],
                                             [np.sin(theta), np.cos(theta), 0],
                                             [0, 0, 1]
                                            ])

        self.x_step = abs(self.pixel_to_xy(0, 0)[0] - self.pixel_to_xy(1, 0)[0])

        #2d (int) array of pixel coords indexed by grid[v][u]
        self.grid = np.array(occupany_grid.data).reshape((occupany_grid.info.height, occupany_grid.info.width))
        #self.grid = np.load('grid.npy')
        # self.grid = np.load('grid_w_lane.npy')

        #here we are dilating the map in order to avoid cutting corners
        self.grid = erosion(self.grid, disk(6))
        # self.grid = dilation(self.grid, disk(6))

    @property
    def height(self) -> int: return self._height

    @property
    def width(self) -> int: return self._width

    @property
    def resolution(self) -> float: return self._resolution

    def __len__(self) -> int: return self._height * self._width

    def xy_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """
        Converts x,y point in map frame to u,v pixel in occupancy grid
        """
        q = np.array([[x], [y], [0]]) #3x1 x,y,z

        q = q - self.origin_p

        _, _, yaw = euler_from_quaternion(self.origin_o)

        q = np.matmul(self.R_z(-yaw),q)

        q = q / self._resolution

        u, v = q[:2, :]

        return (int(round(u[0])), int(round(v[0])))

    def xyt_to_pixel(self, x: float, y: float, theta: float) -> Tuple[int, int, float]:
        """
        Converts x,y,theta point in map frame to u,v,theta pixel in occupancy grid
        """
        q = np.array([[x], [y], [0]]) #3x1 x,y,z

        q = q - self.origin_p

        _, _, yaw = euler_from_quaternion(self.origin_o)

        q = np.matmul(self.R_z(-yaw),q)

        q = q / self._resolution

        u, v = q[:2, :]

        return (int(round(u[0])), int(round(v[0])), theta + yaw)

    def pixel_to_xy(self, u: int, v: int) -> Tuple[float, float]:
        """
        Converts u,v pixel to x,y point in map frame
        """
        pixel = np.array([[u], [v], [0]])

        pixel = pixel * self._resolution

        _, _, yaw = euler_from_quaternion(self.origin_o)

        q = np.matmul(self.R_z(yaw),pixel)

        q = q + self.origin_p

        x, y = q[:2, :]

        return (x[0], y[0], )

    def pixel_to_xyt(self, u: int, v: int, theta: float) -> Tuple[float, float, float]:
        """
        Converts u,v,theta pixel to x,y,theta point in map frame
        """
        pixel = np.array([[u], [v], [0]])

        pixel = pixel * self._resolution

        _, _, yaw = euler_from_quaternion(self.origin_o)

        q = np.matmul(self.R_z(yaw),pixel)

        q = q + self.origin_p

        x, y = q[:2, :]

        return (x[0], y[0], theta - yaw)

    def is_free(self, u, v) -> bool:
        return self.grid[v][u] == 0

    def astar(self, start: Tuple[float,float], goal: Tuple[float,float]):
        '''
        simple alg taken from
        https://en.wikipedia.org/wiki/A*_search_algorithm
        '''
        start_pose = self.discretize_point(start)
        goal = self.discretize_point(goal)

        #h = lambda x,y: ( (y[0]-x[0])**2 + (y[1]-x[1])**2 )**(1/2)
        h = lambda p0, p1: np.linalg.norm(np.array([p0[0] - p1[0], p0[1] - p1[1]])) # I think this is faster
        #heuristic is just Euclidean distance :(
        # h = lambda x,y: ( (y[0]-x[0])**2 + (y[1]-x[1])**2 )**(1/2) * np.arctan2(y[1]-y[0],x[1]-x[0])
        #arc length

        nodelookup = {}

        start = Node(start_pose,parent=None,gscore=0,fscore=h(start_pose,goal))
        nodelookup[start_pose] = start

        q = PriorityQueue()
        q.put(start)

        while not q.empty():
            node = q.get()

            if node.pose == goal:
                return node.extract_path()

            for n in self.get_neighbors(node.pose):
                try:
                    n_obj = nodelookup[n]
                except KeyError:
                    n_obj = Node(n,parent=node)
                    nodelookup[n] = n_obj

                tentative_gscore = node.gscore + h(node.pose,n_obj.pose)

                if tentative_gscore < n_obj.gscore:
                    n_obj.set_gscore(tentative_gscore)
                    n_obj.set_fscore(tentative_gscore + h(n_obj.pose,goal))
                    if n_obj not in q:
                        q.put(n_obj)

    def bfs(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """
        start: tuple of (x, y) coord in map frame
        goal: tuple of (x, y) coord in map frame

        Returns path from start to goal
        """
        start = self.discretize_point(start)
        goal = self.discretize_point(goal)

        visited = {start}
        queue = deque([start])
        parent = {start: None}

        end = None

        while queue:
            current = queue.popleft()
            if current == goal:
                end = current
                break
            for n in self.get_neighbors(current):
                if n not in visited:
                    visited.add(n)
                    queue.append(n)
                    parent[n] = current

        # if no path was found
        if end not in parent:
            return []

        i = end
        path = [end]
        while i != start:
            i = parent[i]
            path.append(i)

        return path[::-1] #path start -> goal in tuples of x,y point nodes

    def generate_circle(self,point: Tuple[float,float]):
        u,v = self.xy_to_pixel(point)

    def prune_path(self,path):
        '''
        gets rid of unnecessary (low slope) points

        need to fix
        '''
        EPS = 0.01

        p = path[0]

        idx = 1

        while idx != len(path)-1:
            diff = np.array(p) - np.array(path[idx])
            orientation = np.arctan2(diff[1],diff[0])
            diff = np.array([diff[0],diff[1],orientation])
            R = self.R_z(orientation)

            relative = np.matmul(diff,R)
            slope = relative[0]/relative[1]

            try:
                if slope < EPS:
                    path[idx] = 0
                else:
                    p = path[idx]

            except ZeroDivisionError: #one of the slopes are 0 so line is straight
                path[idx] = 0
            idx+=1

        return [i for i in path if i!=0]

    def rrt(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """
        RRT done in continuous space, without discretization
        """

        MAX_DIST = 1

        class Node():
            def __init__(self, path, loc):
                self.path = path
                self.loc = loc
                self.parent = None
                self.children = []

        def dist(loc1, loc2):
            return np.linalg.norm(np.array([loc1[0] - loc2[0], loc1[1] - loc2[1]]))

        def find_nearest(loc, parent):
            min_dist = dist(parent.loc, loc)
            near_node = parent

            for child in parent.children:
                near_child, child_dist = find_nearest(loc, child)

                if child_dist <= min_dist:
                    min_dist = child_dist
                    near_node = near_child

            return near_node, min_dist

        def straight_path(begin, end):
            dy = end[1] - begin[1]
            dx = end[0] - begin[0]

            length = math.sqrt(dist(begin, end))
            if length > MAX_DIST:
                end = (begin[0] + dx / length * MAX_DIST, begin[1] + dy / length * MAX_DIST)

            slope = dy / dx

            step = self.x_step if begin[0] < end[0] else -self.x_step

            path = [end]
            stepped = (end[0] - step, end[1] - slope * step)

            while (stepped[0] > begin[0] and begin[0] < end[0]) or (stepped[0] < begin[0] and begin[0] > end[0]):
                path.append(stepped)
                stepped = (stepped[0] - step, stepped[1] - slope * step)

            return path

        def collision_free(path):
            for point in path:
                u, v = self.xy_to_pixel(point[0], point[1])
                if not self.one_grid[v * self._width + u] == 0:
                    return False

            return True

        def path_to(node): # This might be returning the path backwards
            full_path = node.path

            if node.parent != None:
                full_path = full_path + path_to(node.parent)

            return full_path

        # Add start to the tree
        head = Node([start], start)
        previous = head

        # While previous point was not goal
        samples = 0
        max_samples = 50000
        while previous.loc != goal and samples < max_samples: ## THIS TUPLE COMPARISON MIGHT NOT WORK, IDK

            # Pick a random grid cell or sample goal with set probability
            if random.random() > 0.15:
                while True:
                    target = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))

                    if self.grid[target[1]][target[0]] == 0:
                        break

                target = self.pixel_to_xy(target[0], target[1])
                target = (target[0], target[1], random.uniform(-math.pi, math.pi))
            else:
                target = goal

            # Find the nearest node in the tree
            nearest, _ = find_nearest(target, head)

            # Extend the newest point to the nearest node
            nearest_path, act_target = straight_path(nearest.loc, target)

            # If the extended path has no collisions
            if collision_free(nearest_path):

                # Add the newest point to the tree
                newest_node = Node(nearest_path, act_target)
                newest_node.parent = nearest
                nearest.children.append(newest_node)

                # Set this new point as our previous node
                previous = newest_node

            samples += 1

        if samples < max_samples:
            return path_to(previous)
        else:
            return None

    def rrt_star(self, start: Tuple[float, float, float], goal: Tuple[float, float, float]):

        # Constants
        M_TO_PIX = 1 / self._resolution     # Converts meters (x, y) to pixels (u, v)
        GOAL_THRESH = (0.5 * M_TO_PIX)**2   # Distance to goal for termination
        MIN_TURN = 0.4 * M_TO_PIX           # Minimum turn radius (determined by car)

        # Parameters
        MAX_LENGTH = 1 * M_TO_PIX           # Max segment length
        MEAN_TURN = 1.5 * M_TO_PIX          # Average turn radius (gaussian)
        MAX_SEARCH_RADIUS = 5 * M_TO_PIX    # Maximum distance to rewire nodes
        SAMPLE_SIZE = 0.25 * M_TO_PIX       # How often Dubins are sampled
        MAX_SAMPLES = 20000                 # Num of samples before termination

        class Node():
            def __init__(self, path, loc, parent=None):
                self.path = path
                self.loc = loc
                self.parent = parent
                self.cost = 0.0 if parent is None else parent.cost

        def steer(begin, end):

            # Generate the dubins path between the points
            path = dubins.shortest_path(begin, end, max(MEAN_TURN * np.random.normal(loc=1.0), MIN_TURN))
            configurations, _ = path.sample_many(SAMPLE_SIZE)

            # Interpolate the end point if required
            if len(configurations) * SAMPLE_SIZE > MAX_LENGTH:
                configurations = configurations[:int(MAX_LENGTH / SAMPLE_SIZE)]
                end = configurations[-1]

            return configurations, end

        def collision_free(path):

            # Broad phase detection check middle point
            point = path[len(path) // 2]
            if not self.one_grid[int(point[1]) * self._width + int(point[0])] == 0:
                return False

            # Check every point along the path and return false if collision
            for point in reversed(path):
                if not self.one_grid[int(point[1]) * self._width + int(point[0])] == 0:
                    return False

            return True

        def rewire(nodes, new_node):
            n = len(nodes)
            radius = min((self.gamma_estimate * np.log10(n) / n)**(1/3), MAX_SEARCH_RADIUS)

            # Find all nodes within our rewire distance
            near_nodes = [node for node in nodes if (node.loc[0] - new_node.loc[0])**2 + (node.loc[1] - new_node.loc[1]) < radius**2]

            # For every node within the radius
            for node in near_nodes:

                # Ignore our new node
                if node.loc != new_node.loc:

                    # Generate a dubins path between these nodes
                    path = dubins.shortest_path(node.loc, new_node.loc, max(MEAN_TURN * np.random.normal(loc=1.0), MIN_TURN))
                    configurations, _ = path.sample_many(SAMPLE_SIZE)

                    # No collisions along the new path
                    if collision_free(configurations):

                        # Find the cost for this node
                        new_cost = node.cost + len(configurations) * SAMPLE_SIZE

                        # If this node cost is better than the old cost
                        if new_cost < new_node.cost:
                            new_node.parent = node
                            new_node.path = configurations
                            new_node.cost = new_cost

        def path_to(node):
            path = []

            while node is not None:
                cur_path = []
                for point in node.path:
                    point_m = self.pixel_to_xyt(int(point[0]), int(point[1]), point[2])
                    cur_path.append(point_m)

                path = cur_path + path
                node = node.parent

            return path

        ### END OF HELPER FUNCTIONS ###

        # Convert start and goal to the grid
        start = self.xyt_to_pixel(start[0], start[1], start[2])
        goal = self.xyt_to_pixel(goal[0], goal[1], goal[2])

        # Add start to the tree
        nodes = [Node([start], start)]

        # For the maximum number of samples
        for _ in range(MAX_SAMPLES):

            # Pick a random grid cell or sample goal with set probability
            if random.random() > 0.1:

                # Sample a random point
                target = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
                target = (target[0], target[1], random.uniform(0, 2 * math.pi))
            else:
                target = goal

            # Find the nearest node in the list by euclid distance
            nearest_node = min(nodes, key=lambda node: (node.loc[0] - target[0])**2 + (node.loc[1] - target[1])**2)

            # Extend the target to the nearest node
            nearest_path, target = steer(nearest_node.loc, target)

            # If the extended path has no collisions
            if collision_free(nearest_path):

                # Add the newest node to the list
                newest_node = Node(nearest_path, target, parent=nearest_node)
                newest_node.cost += len(nearest_path) * SAMPLE_SIZE
                nodes.append(newest_node)

                # Rewire the tree as necessary
                rewire(nodes, newest_node)

                # If close enough to the goal, return
                if (newest_node.loc[0] - goal[0])**2 + (newest_node.loc[1] - goal[1])**2 <= GOAL_THRESH:

                    # all_nodes = []
                    # for n in nodes:
                    #     point_m = self.pixel_to_xy(int(n.loc[0]), int(n.loc[1]))
                    #     all_nodes.append((point_m[0], point_m[1]))

                    return path_to(newest_node)

        all_nodes = []
        for n in nodes:
            point_m = self.pixel_to_xy(int(n.loc[0]), int(n.loc[1]))
            all_nodes.append((point_m[0], point_m[1]))

        return None

    def rrt_star_reverse(self, start: Tuple[float, float, float], goal: Tuple[float, float, float]):

        # Constants
        M_TO_PIX = 1 / self._resolution     # Converts meters (x, y) to pixels (u, v)
        GOAL_THRESH = (0.75 * M_TO_PIX)**2  # Distance to goal for termination
        ANGLE_THRESH = 0.174533             # How close the final angle must be to goal
        MIN_TURN = 0.4 * M_TO_PIX           # Minimum turn radius (determined by car)

        # Parameters
        MAX_LENGTH = 3 * M_TO_PIX           # Max segment length
        MEAN_TURN = 1.5 * M_TO_PIX          # Average turn radius (gaussian)
        MAX_SEARCH_RADIUS = 3 * M_TO_PIX    # Maximum distance to rewire nodes
        SAMPLE_SIZE = 0.2 * M_TO_PIX        # How often Dubins are sampled
        MAX_SAMPLES = 20000                 # Num of samples before termination

        class Node():
            def __init__(self, path, loc, parent=None):
                self.path = path
                self.loc = loc
                self.parent = parent
                self.cost = 0.0 if parent is None else parent.cost

        def steer(begin, end):

            # Generate the reeds-shepp path between the points
            path = rs.path(
                begin, end, max(MEAN_TURN * np.random.normal(loc=1.0), MIN_TURN), 0, SAMPLE_SIZE
            )
            configurations = path.waypoints()

            # Interpolate the end point if required
            if len(configurations) * SAMPLE_SIZE > MAX_LENGTH:
                configurations = configurations[:int(MAX_LENGTH / SAMPLE_SIZE)]
                end = configurations[-1]

            return configurations, end

        def collision_free(path):

            # Broad phase detection check middle point
            point = path[len(path) // 2]
            if not self.one_grid[int(point[1]) * self._width + int(point[0])] == 0:
                return False

            # Check every point along the path and return false if collision
            for point in reversed(path):
                if not self.one_grid[int(point[1]) * self._width + int(point[0])] == 0:
                    return False

            return True

        def rewire(nodes, new_node):
            n = len(nodes)
            radius = min((self.gamma_estimate * np.log10(n) / n)**(1/3), MAX_SEARCH_RADIUS)

            # Find all nodes within our rewire distance
            near_nodes = [node for node in nodes if (node.loc[0] - new_node.loc[0])**2 + (node.loc[1] - new_node.loc[1]) < radius**2]

            # For every node within the radius
            for node in near_nodes:

                # Ignore our new node
                if node.loc != new_node.loc:

                    # Generate the reeds-shepp path between the points
                    path = rs.path(
                        node.loc, new_node.loc, max(MEAN_TURN * np.random.normal(loc=1.0), MIN_TURN), 0, SAMPLE_SIZE
                    )
                    configurations = path.waypoints()

                    # No collisions along the new path
                    if collision_free(configurations):

                        # Find the cost for this node
                        new_cost = node.cost + len(configurations) * SAMPLE_SIZE

                        # If this node cost is better than the old cost
                        if new_cost < new_node.cost:
                            new_node.parent = node
                            new_node.path = configurations
                            new_node.cost = new_cost

        def path_to(node):
            path = []

            while node is not None:
                cur_path = []
                for point in node.path:
                    point_m = self.pixel_to_xyt(int(point[0]), int(point[1]), point[2])
                    cur_path.append(point_m)

                path = cur_path + path
                node = node.parent

            return path

        ### END OF HELPER FUNCTIONS ###

        # Convert start and goal to the grid
        start = self.xyt_to_pixel(start[0], start[1], start[2])
        goal = self.xyt_to_pixel(goal[0], goal[1], goal[2])

        # Add start to the tree
        nodes = [Node([start], start)]

        # For the maximum number of samples
        for _ in range(MAX_SAMPLES):

            # Pick a random grid cell or sample goal with set probability
            if random.random() > 0.1:

                # Sample a random point
                target = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
                target = (target[0], target[1], random.uniform(0, 2 * math.pi))
            else:
                target = goal

            # Find the nearest node in the list by euclid distance
            nearest_node = min(nodes, key=lambda node: (node.loc[0] - target[0])**2 + (node.loc[1] - target[1])**2)

            # Extend the target to the nearest node
            nearest_path, target = steer(nearest_node.loc, target)

            # If the extended path has no collisions
            if collision_free(nearest_path):

                # Add the newest node to the list
                newest_node = Node(nearest_path, target, parent=nearest_node)
                newest_node.cost += len(nearest_path) * SAMPLE_SIZE
                nodes.append(newest_node)

                # Rewire the tree as necessary
                rewire(nodes, newest_node)

                # If close enough to the goal, return
                if (newest_node.loc[0] - goal[0])**2 + (newest_node.loc[1] - goal[1])**2 <= GOAL_THRESH and abs(newest_node.loc[2] - goal[2]) <= ANGLE_THRESH:

                    # all_nodes = []
                    # for n in nodes:
                    #     point_m = self.pixel_to_xy(int(n.loc[0]), int(n.loc[1]))
                    #     all_nodes.append((point_m[0], point_m[1]))

                    return path_to(newest_node)

        all_nodes = []
        for n in nodes:
            point_m = self.pixel_to_xy(int(n.loc[0]), int(n.loc[1]))
            all_nodes.append((point_m[0], point_m[1]))

        return None

    def get_neighbors(self, point: Tuple[float, float]) -> List[Tuple[float, float]]:
        x, y = point
        neighbors = []

        # radius = 8
        step = 0.5
        possibilities = [(-step, 0), (0, step), (step, 0), (0, -step), (step, step), (step, -step), (-step, step), (-step, -step)]
        for (dx, dy) in possibilities:
            u, v = self.xy_to_pixel(x + dx, y + dy)
            if not self.is_free(u, v):
                neighbors = []
                break
            if (0 <= u and u < self._width) and (0 <= v and v < self._height) and self.is_free(u,v):
                neighbors.append((x + dx, y + dy))

        for (dx, dy) in possibilities:
            u, v = self.xy_to_pixel(x + dx/2, y + dy/2)
            if (0 <= u and u < self._width) and (0 <= v and v < self._height) and self.is_free(u,v):
                neighbors.append((x + dx/2, y + dy/2))



        # radius = 4
        # step = 0.5
        # possibilities = [(-step, 0), (0, step), (step, 0), (0, -step), (step, step), (step, -step), (-step, step), (-step, -step)]
        # for i, (dx, dy) in enumerate(possibilities):
        #     u, v = self.xy_to_pixel(x + dx, y + dy)
        #     if not self.is_free(u, v):
        #         break
        #     if (0 <= u and u < self._width) and (0 <= v and v < self._height) and self.is_free(u,v):
        #         neighbors.append((x + dx, y + dy))


        # cuts corner on path pruning
        # radius = 7
        # step = 1.0
        # possibilities = [(-step, 0), (0, step), (step, 0), (0, -step), (step, step), (step, -step), (-step, step), (-step, -step)]
        # for (dx, dy) in possibilities:
        #     u, v = self.xy_to_pixel(x + dx, y + dy)
        #     if not self.is_free(u, v):
        #         u, v = self.xy_to_pixel(x + dx/2, y + dy/2)
        #         if (0 <= u and u < self._width) and (0 <= v and v < self._height) and self.is_free(u,v):
        #             neighbors.append((x + dx/2, y + dy/2))

        #     elif (0 <= u and u < self._width) and (0 <= v and v < self._height):
        #         neighbors.append((x + dx, y + dy))

        return neighbors

    def discretize_point(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Discretizes a point (x, y) to a node in the space such that nodes are the
        center of each 1x1 grid square. In the case where a point is on the edge
        of a grid square, the point will be assigned to the center with decreasing
        x and increasing y.
        """
        x, y = point

        mid = 0.5

        new_x = int(x)
        new_y = int(y)

        if x >= 0 and y >= 0:
            if x - new_x == 0:
                new_x -= mid
            else:
                new_x += mid

            new_y += mid

        elif x < 0 and y >= 0:
            new_x -= mid
            new_y += mid


        elif x <= 0 and y < 0:
            if y - new_y == 0:
                new_y += mid
            else:
                new_y -= mid

            new_x -= mid

        elif x > 0 and y < 0:
            if y - new_y == 0:
                new_y += mid
            else:
                new_y -= mid

            if x - new_x == 0:
                new_x -= mid
            else:
                new_x += mid

        return (new_x, new_y)

if __name__ == "__main__":
	main()
