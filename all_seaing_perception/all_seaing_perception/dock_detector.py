#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Odometry, PointCloud2
import numpy as np
from sklearn.neighbors import NearestNeighbors
import sensor_msgs_py.point_cloud2 as pc2
from pyminiply import read

class DockDetector(Node):

    def __init__(self):
        super().__init__("dock_detection")

        # Subscribers and publishers
        
        # self.img_pub = self.create_publisher(Image, "image/segmented", 5)
       
        self.odometry_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_cb, 10
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "point_cloud", self.point_cloud_cb, qos_profile_sensor_data
        )
        
        # self.declare_parameter("lidar_point_cloud", "")
        # self.declare_parameter('dock_point_cloud_file', '')
        # dock_point_cloud_file = self.get_parameter('dock_point_cloud_file').value

        self.known_dock_pc, _ = read("point_clouds/roboboat_dock.ply", False, False, False)
        self.lidar_point_cloud = None
        self.robot_pos = (0, 0)
        self.timer = self.create_timer(1, self.detect_dock)

        # vertices, _ = read(dock_point_cloud_file, False, False, False)
        # self.known_dock_pc = vertices
    def nearest_neighbor(src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nxm array of points
            dst: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: dst indices of the nearest neighbor
        '''
        assert src.shape == dst.shape
        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()
    def best_fit_transform(A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
        A: Nxm numpy array of corresponding points
        B: Nxm numpy array of corresponding points
        Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
        R: mxm rotation matrix
        t: mx1 translation vector
        '''
        assert A.shape == B.shape
        # get number of dimensions
        m = A.shape[1]
        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B
        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        # special reflection case
        if np.linalg.det(R) < 0:
            Vt[m-1,:] *= -1
        R = np.dot(Vt.T, U.T)
        # translation
        t = centroid_B.T - np.dot(R,centroid_A.T)
        # homogeneous transformation
        T = np.identity(m+1)
        T[:m, :m] = R
        T[:m, m] = t
        return T, R, t
    def get_point_cloud_transform(self, point_cloud_1, point_cloud_2):
        # Convert 
        # use ICP here
        # return transform and confidence
        return self.icp(point_cloud_1, point_cloud_2)

    def detect_dock(self):
        self.get_logger().info('DETECT DOCK')

        transform, distances, _ = self.get_point_cloud_transform(self.known_dock_pc, self.lidar_point_cloud)
        self.get_logger().info('got point cloud transform')
        # self.lidar_point_cloud_flat = # flatten
        # self.get_point_cloud_transform(__, __)
        # publish something
    def icp(self, A, B, init_pose=None, max_iterations=20, tolerance=0.001):
        '''
        https://github.com/ClayFlannigan/icp/
        
        The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
        Input:
            A: Nxm numpy array of source mD points
            B: Nxm numpy array of destination mD point
            init_pose: (m+1)x(m+1) homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation that maps A on to B
            distances: Euclidean distances (errors) of the nearest neighbor
            i: number of iterations to converge
        '''
        assert A.shape == B.shape
        # get number of dimensions
        m = A.shape[1]
        # make points homogeneous, copy them to maintain the originals
        src = np.ones((m+1,A.shape[0]))
        dst = np.ones((m+1,B.shape[0]))
        src[:m,:] = np.copy(A.T)
        dst[:m,:] = np.copy(B.T)
        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)
        prev_error = 0
        for i in range(max_iterations):
            # find the nearest neighbors between the current source and destination points
            distances, indices = self.nearest_neighbor(src[:m,:].T, dst[:m,:].T)
            # compute the transformation between the current source and nearest destination points
            T,_,_ = self.best_fit_transform(src[:m,:].T, dst[:m,indices].T)
            # update the current source
            src = np.dot(T, src)
            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error
        # calculate final transformation
        T,_,_ = self.best_fit_transform(A, src[:m,:].T)
        return T, distances, i
    
    
    def odometry_cb(self, msg):
        """
        Update the stored robot's position based on the odometry messages
        """
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    def point_cloud_cb(self, msg):
        self.lidar_point_cloud = pc2.read_points_numpy(msg)


def main(args=None):
    rclpy.init(args=args)
    dock_detector = DockDetector()
    rclpy.spin(dock_detector)
    dock_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
