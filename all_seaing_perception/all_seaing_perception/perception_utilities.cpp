#include "all_seaing_perception/perception_utilities.hpp"

namespace all_seaing_perception{
    cv::Point2d project3dToPixel(image_geometry::PinholeCameraModel& cmodel, const cv::Point3d& xyz){
        cv::Point2d uv_rect;
        uv_rect.x = (cmodel.fx()*xyz.x + cmodel.Tx()) / xyz.z + cmodel.cx();
        uv_rect.y = (cmodel.fy()*xyz.y + cmodel.Ty()) / xyz.z + cmodel.cy();
        return uv_rect;
        // return cmodel.project3dToPixel(xyz);
    }

    bool inBounds(image_geometry::PinholeCameraModel& cmodel, const cv::Point2d& xy_rect){
        return (xy_rect.x >= 0) && (xy_rect.x < cmodel.cameraInfo().width) && (xy_rect.y >= 0) &&
            (xy_rect.y < cmodel.cameraInfo().height);
    }

    bool inBBox(const cv::Point2d& xy_rect, all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox){
        return xy_rect.x >= bbox.min_x && xy_rect.x <= bbox.max_x && xy_rect.y >= bbox.min_y && xy_rect.y <= bbox.max_y;
    }

    void addBBoxPadding(all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox, int margin, int height, int width){
        bbox.min_x -= margin;
        bbox.max_x += margin;
        bbox.min_y -= margin;
        bbox.max_y += margin;
        bbox.min_x = std::max((int)bbox.min_x,0);
        bbox.max_x = std::min((int)bbox.max_x+1, width);
        bbox.min_y = std::max((int)bbox.min_y,0);
        bbox.max_y = std::min((int)bbox.max_y+1, height);
    }

    void invertHSVPCL(pcl::PointXYZHSV& pt){
        pt.h = ((int)pt.h+180)%360;
    }

    std::pair<cv::Mat, std::vector<std::vector<cv::Point>>> colorSegmentationHSV(cv::Mat& img, std::vector<int> lims, int erode_sz, int dilate_sz, bool red, std::vector<int> lims2){
        int h_min=lims[0], h_max=lims[1], s_min=lims[2], s_max=lims[3], v_min=lims[4], v_max=lims[5];
        cv::Mat mask;
        cv::inRange(img, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
        if (red){
            // add the pixels that have the other side of the red H range
            cv::Mat mask2;
            int h_min2=lims2[0], h_max2=lims2[1], s_min2=lims2[2], s_max2=lims2[3], v_min2=lims2[4], v_max2=lims2[5];
            cv::inRange(img, cv::Scalar(h_min2, s_min2, v_min2), cv::Scalar(h_max2, s_max2, v_max2), mask2);
            mask += mask2;
        }
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_sz, erode_sz)));
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_sz, dilate_sz)));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        return std::make_pair(mask, contours);
    }

    std::vector<cv::Point> inContour(std::vector<cv::Point>& contour){
        // TAKE THE BOUNDING BOX (BOUNDINGRECT), TAKE ALL THE PIXELS IN IT (NOT NOT CHECK EVERY PIXEL IN THE IMAGE) AND CHECK IF IT'S INSIDE THE CONTOUR USING POINTPOLYGONTEST
        cv::Rect cont_bbox = cv::boundingRect(contour);
        std::vector<cv::Point> in_contour;
        for(int x = cont_bbox.x; x < cont_bbox.x + cont_bbox.width; x++){
            for(int y = cont_bbox.y; y < cont_bbox.y + cont_bbox.height; y++){
                cv::Point pt = cv::Point(x,y);
                if(cv::pointPolygonTest(contour, pt, false)>=0){
                    in_contour.push_back(pt);
                }
            }
        }
        return in_contour;
    }
}