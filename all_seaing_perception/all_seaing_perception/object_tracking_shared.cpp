#include "all_seaing_perception/object_tracking_shared.hpp"

namespace all_seaing_perception{

    ObjectCloud::ObjectCloud(rclcpp::Time t, int l, pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc,
                         pcl::PointCloud<pcl::PointXYZHSV>::Ptr glob) {
        id = -1;
        label = l;
        time_seen = t;
        last_dead = rclcpp::Time(0);
        time_dead = rclcpp::Duration(0, 0);
        is_dead = false;
        global_pcloud_ptr = glob;
        for (pcl::PointXYZHSV &global_pt : global_pcloud_ptr->points) {
            global_centroid.x += global_pt.x / ((double)global_pcloud_ptr->points.size());
            global_centroid.y += global_pt.y / ((double)global_pcloud_ptr->points.size());
            global_centroid.z += global_pt.z / ((double)global_pcloud_ptr->points.size());
        }
        this->update_loc_pcloud(loc);
    }

    void ObjectCloud::update_loc_pcloud(pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc) {
        local_pcloud_ptr = loc;
        for (pcl::PointXYZHSV &local_pt : local_pcloud_ptr->points) {
            local_centroid.x += local_pt.x / ((double)local_pcloud_ptr->points.size());
            local_centroid.y += local_pt.y / ((double)local_pcloud_ptr->points.size());
            local_centroid.z += local_pt.z / ((double)local_pcloud_ptr->points.size());
        }
    }

    std::shared_ptr<ObjectCloud> clone(std::shared_ptr<ObjectCloud> orig){
        std::shared_ptr<ObjectCloud> new_ocl = std::make_shared<ObjectCloud>(*orig);
        new_ocl->local_pcloud_ptr = (*orig->local_pcloud_ptr).makeShared();
        new_ocl->global_pcloud_ptr = (*orig->global_pcloud_ptr).makeShared();
        return new_ocl;
    }

    std::vector<std::shared_ptr<ObjectCloud>> clone(std::vector<std::shared_ptr<ObjectCloud>> orig){
        std::vector<std::shared_ptr<ObjectCloud>> new_vec;
        for(auto ocl : orig){
            new_vec.push_back(clone(ocl));
        }
        return new_vec;
    }

    std::tuple<double, double, double> compute_transform_from_to(double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta){
        double dx = cos(from_theta)*(to_x-from_x)+sin(from_theta)*(to_y-from_y);
        double dy = -sin(from_theta)*(to_x-from_x)+cos(from_theta)*(to_y-from_y);
        double dtheta = to_theta - from_theta;
        // RCLCPP_INFO(this->get_logger(), "(%lf, %lf, %lf) -> (%lf, %lf, %lf): (%lf, %lf, %lf)", from_x, from_y, from_theta, to_x, to_y, to_theta, dx, dy, dtheta);
        return std::make_tuple(dx, dy, dtheta);
    }

    // multiply left to right
    std::tuple<double, double, double> compose_transforms(std::tuple<double, double, double> t1, std::tuple<double, double, double> t2){
        double t1_dx, t1_dy, t1_dtheta, t2_dx, t2_dy, t2_dtheta;
        std::tie(t1_dx, t1_dy, t1_dtheta) = t1;
        std::tie(t2_dx, t2_dy, t2_dtheta) = t2;
        double t_dx = t1_dx+cos(t1_dtheta)*t2_dx-sin(t1_dtheta)*t2_dy;
        double t_dy = t1_dy+sin(t1_dtheta)*t2_dx+cos(t1_dtheta)*t2_dy;
        double t_dtheta = t1_dtheta+t2_dtheta;
        // RCLCPP_INFO(this->get_logger(), "(%lf, %lf, %lf)@(%lf, %lf, %lf)=(%lf, %lf, %lf)", t1_dx, t1_dy, t1_dtheta, t2_dx, t2_dy, t2_dtheta, t_dx, t_dy, t_dtheta);
        return std::make_tuple(t_dx, t_dy, t_dtheta);
    }

    // given the starting and ending robot positions (wrt to world) and object pos wrt world given first robot position being true, compute world to object given second robot position being true (robot to object should be the same)
    // useful to overlay maps with two different robot positions
    std::tuple<double, double, double> apply_transform_from_to(double x, double y, double theta, double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta){
        std::tuple<double, double, double> local_rel = compute_transform_from_to(from_x, from_y, from_theta, x, y, theta);// from starting pos to object
        return compose_transforms(std::make_tuple(to_x, to_y, to_theta), local_rel);
    }

    double mod_2pi(double angle){ // pushes angle to [0,2pi) range
        return angle >= 2*M_PI?angle-floor(angle/(2*M_PI))*2*M_PI:angle+ceil(-angle/(2*M_PI))*2*M_PI;
    }

    double angle_to_pi_range(double angle){ // pushes angle to [-pi, pi) range
        return mod_2pi(angle+M_PI)-M_PI;
    }

    //(range, bearing, signature)
    std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZ point, int label) {
        double range = std::hypot(point.x, point.y);
        double bearing = mod_2pi(std::atan2(point.y, point.x));
        // RCLCPP_INFO(this->get_logger(), "LOCAL: (%lf, %lf) -> RBS: (%lf, %lf, %d)", point.x, point.y,
        // range, bearing, label);
        return std::make_tuple(range, bearing, label);
    }

    geometry_msgs::msg::TransformStamped get_tf(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer, const std::string &in_target_frame,
                                                                const std::string &in_src_frame) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            // RCLCPP_INFO(this->get_logger(), "%s", ex.what());
        }
        return tf;
    }

    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(std::vector<std::shared_ptr<ObjectCloud>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres){
        // Assign each detection to a tracked or new object using the computed squared Mahalanobis distance
        std::vector<int> match(detected_obstacles.size(), -1);
        float min_p = 0;
        std::unordered_set<int> chosen_detected, chosen_tracked;
        while (min_p < new_obj_thres) {
            min_p = new_obj_thres;
            std::pair<int, int> best_match = std::make_pair(-1, -1);
            for (int i = 0; i < detected_obstacles.size(); i++) {
                if (chosen_detected.count(i))
                    continue;
                for (int tracked_id = 0; tracked_id < tracked_obstacles.size(); tracked_id++) {
                    if (chosen_tracked.count(tracked_id) ||
                        tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label)
                        continue;
                    if (p[i][tracked_id] < min_p) {
                        best_match = std::make_pair(i, tracked_id);
                        min_p = p[i][tracked_id];
                    }
                }
            }
            if (min_p < new_obj_thres) {
                match[best_match.first] = best_match.second;
                chosen_tracked.insert(best_match.second);
                chosen_detected.insert(best_match.first);
            }
        }
        return std::make_tuple(match, chosen_detected, chosen_tracked);
    }

    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(std::vector<std::shared_ptr<ObjectCloud>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres){
        // Assign each detection to a tracked or new object using the computed squared Mahalanobis distance
        std::vector<int> match(detected_obstacles.size(), -1);
        float min_p = 0;
        std::unordered_set<int> chosen_detected, chosen_tracked;
        for (int i = 0; i < detected_obstacles.size(); i++) {
            min_p = new_obj_thres;
            int best_match = -1;
            for (int tracked_id = 0; tracked_id < tracked_obstacles.size(); tracked_id++) {
                if (tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label)
                    continue;
                if (p[i][tracked_id] < min_p) {
                    best_match = tracked_id;
                    min_p = p[i][tracked_id];
                }
            }
            if (min_p < new_obj_thres) {
                match[i] = best_match;
                chosen_tracked.insert(best_match);
                chosen_detected.insert(i);
            }
        }
        return std::make_tuple(match, chosen_detected, chosen_tracked);
    }

    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(std::vector<std::shared_ptr<ObjectCloud>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres, bool sqrt){
        // Assign each detection to a tracked or new object using the computed squared Mahalanobis distance
        // sqrt is whether we are using the Mahalanobis distances (sqrt of p values) and not their squares (p values)
        std::vector<int> match(detected_obstacles.size(), -1);
        std::unordered_set<int> chosen_detected, chosen_tracked;
        int num_det = detected_obstacles.size();
        int num_obj = tracked_obstacles.size();
        const int INF = 1e9;
        Eigen::MatrixXf cost_matrix = Eigen::MatrixXf::Zero(num_det+num_obj, num_obj+num_det); // first dim detections + non-assignments of obstacles, second dim obstacles + non-assignments of detections
        float max_val = 0;
        for (int i = 0; i < num_det; i++) {
            for (int tracked_id = 0; tracked_id < num_obj; tracked_id++) {
                if (tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label || p[i][tracked_id] >= new_obj_thres){
                    cost_matrix(i,tracked_id) = INF;
                }else{
                    cost_matrix(i,tracked_id) = sqrt ? std::sqrt(p[i][tracked_id]) : p[i][tracked_id];
                    max_val = std::max(max_val, cost_matrix(i,tracked_id));
                }
            }
        }
        cost_matrix.block(num_det, 0, num_obj, num_obj).fill(max_val+1);
        cost_matrix.block(0, num_obj, num_det+num_obj, num_det).fill(max_val+1);

        // now we have an appropriate cost matrix
        // run the Hungarian algorithm or JVC which is faster

        // convert to vector of vectors
        std::vector<std::vector<double>> cost_matrix_vec;
        for (int i = 0; i < num_det + num_obj; i++) {
            cost_matrix_vec.push_back(std::vector<double>());
            for (int j = 0; j < num_obj + num_det; j++){
                cost_matrix_vec[i].push_back(cost_matrix(i,j));
            }
        }

        HungarianAlgorithm HungAlgo;
        vector<int> assignment;

        double cost = HungAlgo.Solve(cost_matrix_vec, assignment);

        for (int i = 0; i < num_det; i++){
            if (assignment[i] < num_obj && cost_matrix(i, assignment[i]) != INF){
                match[i] = assignment[i];
            }
        }

        return std::make_tuple(match, chosen_detected, chosen_tracked);
    }
    
    std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(std::vector<std::shared_ptr<ObjectCloud>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> probs, float new_obj_thres){
        // Compute weight of particle based on the probability of the correspondence of matched obstacles (detections<->map)
        // which is the product of the probabilities of each detection given the measurement prediction and covariance
        // (computed by the EKFs of the individual obstacles) 
        std::vector<int> match;
        std::unordered_set<int> chosen_detected, chosen_tracked;
        std::tie(match, chosen_detected, chosen_tracked) = greedy_data_association(tracked_obstacles, detected_obstacles, p, new_obj_thres);
        // probs' last element has the probability that an object is newly detected, computed using the mahalanobis distance threshold and the covariance matrix
        float weight = 1;
        for (size_t i = 0; i < detected_obstacles.size(); i++) {
            if(match[i] == -1){
                //unassigned
                weight *= probs[i].back();
            }else{
                weight *= probs[i][match[i]];
            }
        }
        return std::make_tuple(weight, match, chosen_detected, chosen_tracked); 
    }

    void publish_map(
        std_msgs::msg::Header local_header, std_msgs::msg::Header global_header, std::string ns, bool is_labeled,
        const std::vector<std::shared_ptr<Obstacle>> &map,
        rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub,
        std::vector<int> labels) {
    
        all_seaing_interfaces::msg::ObstacleMap map_msg;
        map_msg.ns = ns;
        map_msg.local_header = local_header;
        map_msg.header = global_header;
        map_msg.is_labeled = is_labeled;
        for (size_t i = 0; i < map.size(); i++) {
            all_seaing_interfaces::msg::Obstacle det_obstacle;
            map[i]->to_ros_msg(det_obstacle);
            if (is_labeled) {
                det_obstacle.label = labels[i];
            }
            map_msg.obstacles.push_back(det_obstacle);
        }
        pub->publish(map_msg);
    }

}