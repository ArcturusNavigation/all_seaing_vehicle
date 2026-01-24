import math
import sensor_msgs_py.point_cloud2 as pc2


class PotentialField:
    def __init__(self, pointcloud, q=2.0, goal=None, k=None):
        self.pointcloud = pointcloud   # list of obstacle positions [x, y]
        self.lidar_point_cloud = pc2.read_points_numpy(pointcloud)
        self.goal = goal               # [x, y]
        self.q = q                     # influence distance for repulsion
        self.k = min(k, len(self.lidar_point_cloud)) if (k is not None) else len(self.lidar_point_cloud) # number of nearest obstacles to consider

    def dist(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def potential(self, pos):
        # attractive potential
        U_att = self.dist(pos, self.goal)**2

        # repulsive potential
        obstacles = self.closest_obstacles(pos, self.k)
        U_rep = 0
        for obs in obstacles:
            U_rep += max(1/self.dist(pos, obs)**2 - 1/self.q**2, 0)
        return U_att + U_rep

    def closest_obstacles(self, pos, k):
        return sorted(self.lidar_point_cloud, key=lambda o: self.dist(o, pos))[:k]

    # def robust_gradient_descent_step(self, pos, step_size=0.1, num_samples=36):
    #     # approximate gradient by sampling directions
    #     best_dir = None
    #     best_U = float('inf')

    #     for angle_deg in range(0, 360, int(360/num_samples)):
    #         dx = step_size * math.cos((angle_deg)*math.pi/180)
    #         dy = step_size * math.sin((angle_deg)*math.pi/180)
    #         new_pos = [pos[0] + dx, pos[1] + dy]

    #         U_new = self.potential(new_pos)
    #         if U_new < best_U:
    #             best_U = U_new
    #             best_dir = (dx, dy)

    #     # move a small step in direction of decreasing potential
    #     new_pos = [pos[0] + best_dir[0], pos[1] + best_dir[1]]
    #     return new_pos

    def sketchy_gradient_descent_step(self, pos=(0,0), normalize = False):
        # analytically compute gradient
        points = self.closest_obstacles(pos, self.k)

        partialx = 0
        partialy = 0

        for point in points:
            point_dist = self.dist(point,pos)
            if point_dist >= self.q: # if too far ignore
                break
            partialx+=2*(point_dist**-0.5 - 1/self.q)*(-0.5)*(point_dist**-1.5)*2*(pos[0]-point[0])
            partialy+=2*(point_dist**-0.5 - 1/self.q)*(-0.5)*(point_dist**-1.5)*2*(pos[1]-point[1])
        if self.goal:
            partialx += (pos[0]-self.goal[0])*((pos[0]-self.goal[0])**2 + (pos[1]-self.goal[1]))**-0.5
            partialy += (pos[1]-self.goal[1])*((pos[0]-self.goal[0])**2 + (pos[1]-self.goal[1]))**-0.5
        
        # normalize
        mag = (partialx**2 + partialy**2)**0.5 if normalize else 1
        return [-partialx/mag, -partialy/mag]
    
    def rotational_force(self, pos=(0,0), vel_dir=(1,0), normalize = False):
        # analytically compute gradient
        points = self.closest_obstacles(pos, self.k)

        partialx = 0
        partialy = 0

        for point in points:
            point_dist = self.dist(point,pos)
            if point_dist >= self.q or point[0] < 0: # if obstacle is too far or to the back of the robot, don't add rotational force
                break
            if -vel_dir[1]*point[0]+vel_dir[0]*point[1] > 0: # dot product of vel (in robot's frame) and pos of obstacle
                # obstacle is to the left of the robot velocity's direction, rotational force is CCW

                partialx+=-2*(point_dist**-0.5 - 1/self.q)*(-0.5)*(point_dist**-1.5)*2*(pos[1]-point[1])
                partialy+=2*(point_dist**-0.5 - 1/self.q)*(-0.5)*(point_dist**-1.5)*2*(pos[0]-point[0])
            else:
                # right, rotational force is CW
                partialx+=2*(point_dist**-0.5 - 1/self.q)*(-0.5)*(point_dist**-1.5)*2*(pos[1]-point[1])
                partialy+=-2*(point_dist**-0.5 - 1/self.q)*(-0.5)*(point_dist**-1.5)*2*(pos[0]-point[0])
        
        # normalize
        mag = (partialx**2 + partialy**2)**0.5 if normalize else 1
        return [-partialx/mag, -partialy/mag]


    def run(self, start, max_iters=1000, tol=0.1, step_size=0.1):
        pos = start
        path = [pos]
        for _ in range(max_iters):
            if self.dist(pos, self.goal) < tol:
                break

            
            grad = self.sketchy_gradient_descent_step(pos)
            pos[0]+=grad[0]*step_size
            pos[1]+=grad[1]*step_size
            path.append(pos.copy())
        return path
