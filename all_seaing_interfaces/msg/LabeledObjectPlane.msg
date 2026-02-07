uint32 id
int8 label
geometry_msgs/Pose normal_ctr # represents the normal of the plane, centered at the centroid of int8 -> x is the normal (towards the robot), y is left (horizontal to ground), z is up
geometry_msgs/Vector3 size # size of the plane in y,z axes (x = 0)