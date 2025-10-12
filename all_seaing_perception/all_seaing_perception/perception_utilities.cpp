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

    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance, int obstacle_sz_min, int obstacle_sz_max, bool conditional, std::function<bool(const pcl::PointXYZHSV&, const pcl::PointXYZHSV&, float)> cond_func){
        // extract clusters
        pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZHSV>);
        if (!pcloud_ptr->points.empty())
            tree->setInputCloud(pcloud_ptr);
    
        // EUCLIDEAN CLUSTERING
        if (conditional){
            pcl::ConditionalEuclideanClustering<pcl::PointXYZHSV> ec;
            ec.setClusterTolerance(clustering_distance);
            ec.setMinClusterSize(obstacle_sz_min);
            ec.setMaxClusterSize(obstacle_sz_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcloud_ptr);
            ec.setConditionFunction(cond_func);
            ec.segment(clusters_indices);
        }else{
            pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> ec;
            ec.setClusterTolerance(clustering_distance);
            ec.setMinClusterSize(obstacle_sz_min);
            ec.setMaxClusterSize(obstacle_sz_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcloud_ptr);
            ec.extract(clusters_indices);
        }
    }

    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance, int obstacle_sz_min, int obstacle_sz_max, bool conditional, std::function<bool(const pcl::PointXYZI&, const pcl::PointXYZI&, float)> cond_func){
        // extract clusters
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        if (!pcloud_ptr->points.empty())
            tree->setInputCloud(pcloud_ptr);
    
        // EUCLIDEAN CLUSTERING
        if (conditional){
            pcl::ConditionalEuclideanClustering<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clustering_distance);
            ec.setMinClusterSize(obstacle_sz_min);
            ec.setMaxClusterSize(obstacle_sz_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcloud_ptr);
            ec.setConditionFunction(cond_func);
            ec.segment(clusters_indices);
        }else{
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clustering_distance);
            ec.setMinClusterSize(obstacle_sz_min);
            ec.setMaxClusterSize(obstacle_sz_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcloud_ptr);
            ec.extract(clusters_indices);
        }
    }

    void euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_ptr, std::vector<pcl::PointIndices>& clusters_indices, double clustering_distance, int obstacle_sz_min, int obstacle_sz_max, bool conditional, std::function<bool(const pcl::PointXYZ&, const pcl::PointXYZ&, float)> cond_func){
        // extract clusters
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        if (!pcloud_ptr->points.empty())
            tree->setInputCloud(pcloud_ptr);
    
        // EUCLIDEAN CLUSTERING
        if (conditional){
            pcl::ConditionalEuclideanClustering<pcl::PointXYZ> ec;
            ec.setClusterTolerance(clustering_distance);
            ec.setMinClusterSize(obstacle_sz_min);
            ec.setMaxClusterSize(obstacle_sz_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcloud_ptr);
            ec.setConditionFunction(cond_func);
            ec.segment(clusters_indices);
        }else{
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(clustering_distance);
            ec.setMinClusterSize(obstacle_sz_min);
            ec.setMaxClusterSize(obstacle_sz_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcloud_ptr);
            ec.extract(clusters_indices);
        }
    }

    void PCLInBBoxHSV(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr, all_seaing_interfaces::msg::LabeledBoundingBox2D& bbox, cv::Mat& img, image_geometry::PinholeCameraModel& cmodel, bool is_sim){
        for (pcl::PointXYZI &point_tf : cloud->points) {
            cv::Point2d xy_rect = all_seaing_perception::projectPCLPtToPixel(cmodel, point_tf, is_sim);
            // Check if within bounds & in front of the boat
            if (all_seaing_perception::inBounds(cmodel, xy_rect) && (is_sim? point_tf.x : point_tf.z) >= 0) {      
                // Check if point is in bbox
                if(all_seaing_perception::inBBox(xy_rect, bbox)){
                    std::vector<float> hsv_pcl = all_seaing_perception::HSVOpenCVToPCL<float>(img.at<cv::Vec3b>(xy_rect));
                    obj_cloud_ptr->push_back(pcl::PointXYZHSV(point_tf.x, point_tf.y, point_tf.z, hsv_pcl[0], hsv_pcl[1], hsv_pcl[2]));
                }
            }
        }
    }

    template<typename PointT>
    void transformPCLCloud(const typename pcl::PointCloud<PointT> pcl_in, typename pcl::PointCloud<PointT> &pcl_out, geometry_msgs::msg::TransformStamped tf){
        tf2::Transform tf_tf;
        tf2::fromMsg(tf.transform, tf_tf);
        pcl_ros::transformPointCloud(pcl_in, pcl_out, tf_tf);
        // pcl::transformPointCloud(pcl_in, pcl_out, Eigen::Affine3f(tf2::transformToEigen(tf)));
        pcl_out.header = pcl_in.header;
        pcl_out.header.frame_id = tf.header.frame_id;
    }

    template void transformPCLCloud(const typename pcl::PointCloud<pcl::PointXYZ> pcl_in, typename pcl::PointCloud<pcl::PointXYZ> &pcl_out, geometry_msgs::msg::TransformStamped tf);
    template void transformPCLCloud(const typename pcl::PointCloud<pcl::PointXYZI> pcl_in, typename pcl::PointCloud<pcl::PointXYZI> &pcl_out, geometry_msgs::msg::TransformStamped tf);
    template void transformPCLCloud(const typename pcl::PointCloud<pcl::PointXYZHSV> pcl_in, typename pcl::PointCloud<pcl::PointXYZHSV> &pcl_out, geometry_msgs::msg::TransformStamped tf);
    template void transformPCLCloud(const typename pcl::PointCloud<pcl::PointXYZRGB> pcl_in, typename pcl::PointCloud<pcl::PointXYZRGB> &pcl_out, geometry_msgs::msg::TransformStamped tf);

    template<typename PointT>
    pcl::PointCloud<pcl::PointXYZ> pickLargestCluster(typename pcl::PointCloud<PointT> pcl_in, double clust_dist){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(pcl_in, *pcl_in_ptr);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> clusters_indices;
        euclideanClustering(pcl_in_ptr, clusters_indices, clust_dist);
        int opt_clust = -1;
        for (int i = 0; i < clusters_indices.size(); i++){
            if (opt_clust == -1 || clusters_indices[i].indices.size() > clusters_indices[opt_clust].indices.size()){
                opt_clust = i;
            }
        }
        pcl::copyPointCloud(pcl_in, clusters_indices[opt_clust], *filtered_pcl_ptr);
        return *filtered_pcl_ptr;
    }

    template pcl::PointCloud<pcl::PointXYZ> pickLargestCluster(typename pcl::PointCloud<pcl::PointXYZ> pcl_in, double clust_dist);

    template<typename PointT>
    std::pair<Eigen::Vector3d, Eigen::Vector3d> centroidDims(typename pcl::PointCloud<PointT> pcl_in, Eigen::Matrix3d axes){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(pcl_in, *pcl_in_ptr);

        Eigen::Vector3d ctr, sz;

        Eigen::Vector4f ctr_4;
        pcl::compute3DCentroid(*pcl_in_ptr, ctr_4);
        ctr = ctr_4.head(3).cast<double>();
        
        // Transform to normal frame
        Eigen::Matrix4d projectionTransform(Eigen::Matrix4d::Identity());
        projectionTransform.block<3,3>(0,0) = axes.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * ctr.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*pcl_in_ptr, *cloudPointsProjected, projectionTransform);

        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        sz = Eigen::Vector3d(maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);

        return std::make_pair(ctr, sz);
    }

    template std::pair<Eigen::Vector3d, Eigen::Vector3d> centroidDims(typename pcl::PointCloud<pcl::PointXYZ> pcl_in, Eigen::Matrix3d axes);
    
    template<typename PointT>
    pcl::PointCloud<pcl::PointXYZ> pickInliers(typename pcl::PointCloud<PointT> pcl_in, Eigen::Vector3d ctr, Eigen::Matrix3d normal, double dist_thres){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(pcl_in, *pcl_in_ptr);
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pcl_in_ptr));
        Eigen::VectorXf model_coeff;
        model_coeff.resize(4);
        model_coeff.head(3) = ((Eigen::Vector3d)normal.block<3,1>(0,0)).cast<float>();
        model_coeff[3] = -normal.block<3,1>(0,0).dot(ctr);
        model_p->selectWithinDistance(model_coeff, dist_thres, inliers);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_out_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pcl_in_ptr, inliers, *pcl_out_ptr);
        return *pcl_out_ptr;
    }

    template pcl::PointCloud<pcl::PointXYZ> pickInliers(typename pcl::PointCloud<pcl::PointXYZHSV> pcl_in, Eigen::Vector3d ctr, Eigen::Matrix3d normal, double dist_thres);

    template<typename PointT>
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, int> PCLRANSAC(typename pcl::PointCloud<PointT> pcl_in, double dist_thres, int max_iter, bool cluster, double clust_dist){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(pcl_in, *pcl_in_ptr);

        // Cluster before RANSAC, sometimes pcls of banners of boats inside dock include points from the slot banner, so those get picked up in the plane too
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        if (cluster){
            *filtered_pcl_ptr = pickLargestCluster(*pcl_in_ptr, clust_dist);
        }else{
            filtered_pcl_ptr = pcl_in_ptr;
        }

        if (filtered_pcl_ptr->size() < 3){
            return std::make_tuple(Eigen::Vector3d(), Eigen::Matrix3d(), Eigen::Vector3d(), -1);
        }

        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (filtered_pcl_ptr));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);

        ransac.setDistanceThreshold(dist_thres);
        ransac.setMaxIterations(max_iter);
        ransac.computeModel();
        
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*filtered_pcl_ptr, inliers, *inlier_pcl_ptr);

        Eigen::Vector3d ctr, sz;

        Eigen::Vector3d x_axis, y_axis, z_axis;
        Eigen::VectorXf model_coef;
        ransac.getModelCoefficients(model_coef);
        x_axis = ((Eigen::Vector3f)model_coef.head(3)).cast<double>();
        y_axis = x_axis.cross(Eigen::Vector3d(0,0,1)).normalized();
        z_axis = x_axis.cross(y_axis);
        Eigen::Matrix3d normal;
        normal << x_axis, y_axis, z_axis;
        
        std::tie(ctr, sz) = centroidDims(*inlier_pcl_ptr, normal);

        // Negate x axis & y axis if x axis away from robot (take dot product with centroid coordinates, should be negative)
        // Sizes are the same
        if (normal.col(0).dot(ctr) > 0){
            normal.col(0) *= -1;
            normal.col(1) *= -1;
        }

        return std::make_tuple(ctr, normal, sz, inliers.size());
    }
    
    template std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, int> PCLRANSAC(typename pcl::PointCloud<pcl::PointXYZHSV> pcl_in, double dist_thres, int max_iter, bool cluster, double clust_dist);
    template std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, int> PCLRANSAC(typename pcl::PointCloud<pcl::PointXYZI> pcl_in, double dist_thres, int max_iter, bool cluster, double clust_dist);
}