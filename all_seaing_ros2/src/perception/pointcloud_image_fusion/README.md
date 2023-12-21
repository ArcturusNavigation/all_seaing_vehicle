# pointcloud_image_fusion
The purpose of this package is to project pointcloud data into the given image space

## Process
* `PcImageFusionCb`
Transforms the 3D world coordinates to camera coordinates via extrinsic matrix which consists of a rotation and translation between two coordinate systems. Projects the 3D camera point into the image plane using the intrinsic matrix consisting of the focal length and optical center. 

Subscribe to the image msg and the pointcloud msg to overlay the current pointcloud onto the image. 

## ROS API: pointcloud_image_overlay

### Subs:
* `img_src` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))
Input image to overlay 3D points into

* `img_info_src` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html))
Input image intrinsic data

* `cloud_src` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
Input Point Cloud to project into the 2D image space

### Pubs:
* `/pc_image_fusion` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))
