std_msgs/Header header

uint32 id
#uint32 last_seen  # sweep count since cluster last seen
builtin_interfaces/Time last_seen 
#uint32 type  # 0 obstacle, 1 contact
string label

sensor_msgs/PointCloud2 cloud

geometry_msgs/PointStamped min_point
geometry_msgs/PointStamped max_point
geometry_msgs/PointStamped avg_point
geometry_msgs/PointStamped centroid_point
#geometry_msgs/Twist rel_velocity

#float64 estimated_angle  # Angle to cluster centroid 

geometry_msgs/PolygonStamped convex_hull

#nmea_msgs/Sentence nmea_msg

BoundingBox bb_msg

float32 polygon_area # calculated polygon (convex hull) area
