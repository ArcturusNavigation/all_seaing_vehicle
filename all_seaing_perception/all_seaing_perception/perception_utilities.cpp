#include "all_seaing_perception/perception_utilities.hpp"

namespace all_seaing_perception{
    cv::Point2d project3dToPixel(image_geometry::PinholeCameraModel cmodel, const cv::Point3d& xyz){
        cv::Point2d uv_rect;
        uv_rect.x = (cmodel.fx()*xyz.x + cmodel.Tx()) / xyz.z + cmodel.cx();
        uv_rect.y = (cmodel.fy()*xyz.y + cmodel.Ty()) / xyz.z + cmodel.cy();
        return uv_rect;
        // return cmodel.project3dToPixel(xyz);
    }
}