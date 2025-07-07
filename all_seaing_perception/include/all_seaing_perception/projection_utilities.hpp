#ifndef ALL_SEAING_PERCEPTION__PROJECTION_UTILITIES_HPP
#define ALL_SEAING_PERCEPTION__PROJECTION_UTILITIES_HPP

#include "image_geometry/pinhole_camera_model.h"
#include <opencv2/opencv.hpp>

namespace all_seaing_perception{
    cv::Point2d project3dToPixel(image_geometry::PinholeCameraModel cmodel, const cv::Point3d& xyz);
} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__OBJECT_TRACKING_SHARED_HPP