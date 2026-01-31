#include "all_seaing_perception/object_tracking_shared.hpp"

namespace all_seaing_perception{

    template<typename PointT>
    ObjectCloud<PointT>::ObjectCloud(rclcpp::Time t, int l, typename all_seaing_perception::Obstacle<PointT> obs) : obstacle(obs) {
        label = l;
        time_seen = t;
        last_dead = rclcpp::Time(0);
        time_dead = rclcpp::Duration(0, 0);
        is_dead = false;
    }

    Banner::Banner(rclcpp::Time t, int l, all_seaing_interfaces::msg::LabeledObjectPlane msg){
        label = l;
        time_seen = t;
        last_dead = rclcpp::Time(0);
        time_dead = rclcpp::Duration(0, 0);
        is_dead = false;
        plane_msg = msg;
    }

    template<typename PointT>
    std::shared_ptr<ObjectCloud<PointT>> clone(typename std::shared_ptr<ObjectCloud<PointT>> orig){
        std::shared_ptr<ObjectCloud<PointT>> new_ocl = std::make_shared<ObjectCloud<PointT>>(*orig);
        return new_ocl;
    }

    template<typename PointT>
    std::vector<std::shared_ptr<ObjectCloud<PointT>>> clone(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> orig){
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> new_vec;
        for(auto ocl : orig){
            new_vec.push_back(clone(ocl));
        }
        return new_vec;
    }

    std::tuple<double, double, double> compute_transform_from_to(double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta){
        double dx = cos(from_theta)*(to_x-from_x)+sin(from_theta)*(to_y-from_y);
        double dy = -sin(from_theta)*(to_x-from_x)+cos(from_theta)*(to_y-from_y);
        double dtheta = to_theta - from_theta;
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

    double bidirectional_angle_to_pi_range(double angle){
        double pi_range_angle = angle_to_pi_range(angle);
        return abs(pi_range_angle) < M_PI/((float)2) ? pi_range_angle : angle_to_pi_range(angle+M_PI);
    }

    //(range, bearing, signature)
    template<typename PointT>
    std::tuple<float, float, int> local_to_range_bearing_signature(PointT point, int label) {
        double range = std::hypot(point.x, point.y);
        double bearing = mod_2pi(std::atan2(point.y, point.x));
        return std::make_tuple(range, bearing, label);
    }

    //(range, bearing, phi, signature)
    std::tuple<float, float, float, int> local_banner_to_range_bearing_signature(geometry_msgs::msg::Pose pose, int label){
        double range = std::hypot(pose.position.x, pose.position.y);
        double bearing = mod_2pi(std::atan2(pose.position.y, pose.position.x));
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 mat(q);
        double r, p, phi;
        mat.getRPY(r, p, phi);
        if(r > M_PI/2 || p > M_PI/2){ // to discard weird RPY solutions
            mat.getRPY(r, p, phi, 2);
        }
        return std::make_tuple(range, bearing, phi, label);
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

    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
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
                        (detected_obstacles[i]->label != -1 && tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label))
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

    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_indiv_var_data_association(std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<float> v_indiv, float new_obj_thres){
        // Assign each detection to a tracked or new object using the computed squared Mahalanobis distance
        std::vector<int> match(detected_obstacles.size(), -1);
        std::unordered_set<int> chosen_detected, chosen_tracked;
        std::vector<int> tracked_indices(tracked_obstacles.size(), 0);
        std::iota(tracked_indices.begin(), tracked_indices.end(), 0);
        std::sort(tracked_indices.begin(), tracked_indices.end(), [v_indiv](int a, int b){return v_indiv[a] < v_indiv[b];}); // sort by increasing total variance
        for (int tracked_id : tracked_indices) {
            float min_p = new_obj_thres;
            int best_match = -1;
            for (int i = 0; i < detected_obstacles.size(); i++) {
                if (chosen_detected.count(i) || 
                    (detected_obstacles[i]->label != -1 && tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label))
                    continue;
                if (p[i][tracked_id] < min_p) {
                    best_match = i;
                    min_p = p[i][tracked_id];
                }
            }
            if (min_p < new_obj_thres) {
                match[best_match] = tracked_id;
                chosen_tracked.insert(tracked_id);
                chosen_detected.insert(best_match);
            }
        }
        return std::make_tuple(match, chosen_detected, chosen_tracked);
    }

    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_meas_var_data_association(std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> v_meas, float new_obj_thres){
        // Assign each detection to a tracked or new object using the computed squared Mahalanobis distance
        std::vector<int> match(detected_obstacles.size(), -1);
        std::unordered_set<int> chosen_detected, chosen_tracked;
        std::vector<std::pair<int,int>> pair_ind;
        for (int i = 0; i < detected_obstacles.size(); i++){
            for (int j = 0; j < tracked_obstacles.size(); j++){
                pair_ind.push_back(std::make_pair(i,j));
            }
        }
        std::sort(pair_ind.begin(), pair_ind.end(), [v_meas](std::pair<int,int> a, std::pair<int,int> b){return v_meas[a.first][a.second] < v_meas[b.first][b.second];}); // sort by increasing total variance
        for (std::pair<int,int> p_ind : pair_ind){
            int i = p_ind.first;
            int tracked_id = p_ind.second;
            if (chosen_detected.count(i))
                continue;
            if (chosen_tracked.count(tracked_id) ||
                (detected_obstacles[i]->label != -1 && tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label))
                continue;
            if (p[i][tracked_id] < new_obj_thres) {
                match[i] = tracked_id;
                chosen_detected.insert(i);
                chosen_tracked.insert(tracked_id);
            }
        }
        return std::make_tuple(match, chosen_detected, chosen_tracked);
    }

    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres){
        // Assign each detection to a tracked or new object using the computed squared Mahalanobis distance
        std::vector<int> match(detected_obstacles.size(), -1);
        float min_p = 0;
        std::unordered_set<int> chosen_detected, chosen_tracked;
        for (int i = 0; i < detected_obstacles.size(); i++) {
            min_p = new_obj_thres;
            int best_match = -1;
            for (int tracked_id = 0; tracked_id < tracked_obstacles.size(); tracked_id++) {
                if (detected_obstacles[i]->label != -1 && tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label)
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

    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
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
                if ((detected_obstacles[i]->label != -1 && tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label) || p[i][tracked_id] >= new_obj_thres){
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
    
    template<typename PointT>
    std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
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

    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_banner_data_association(std::vector<std::shared_ptr<Banner>> tracked_obstacles,
        std::vector<std::shared_ptr<Banner>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres,
        std::map<int, int> banner_label_number){
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
                    if (chosen_tracked.count(tracked_id)) continue;
                    if (banner_label_number.count(detected_obstacles[i]->label) && 
                        banner_label_number.count(tracked_obstacles[tracked_id]->label) &&
                        banner_label_number[detected_obstacles[i]->label] == tracked_obstacles[tracked_id]->label){
                        // we're good
                    }else if (detected_obstacles[i]->label != -1 && tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label){
                        continue;
                    }
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

    template class ObjectCloud<pcl::PointXYZ>;
    template class ObjectCloud<pcl::PointXYZI>;
    template class ObjectCloud<pcl::PointXYZHSV>;

    template std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZ point, int label);
    template std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZI point, int label);
    template std::tuple<float, float, int> local_to_range_bearing_signature(pcl::PointXYZHSV point, int label);

    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);

    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_indiv_var_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<float> v_indiv, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_indiv_var_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<float> v_indiv, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_indiv_var_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<float> v_indiv, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_meas_var_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> v_meas, float new_obj_thres);

    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_meas_var_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> v_meas, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_meas_var_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> v_meas, float new_obj_thres);

    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres, bool sqrt);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres, bool sqrt);
    
    template std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres, bool sqrt);
    
    template std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZ>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> probs, float new_obj_thres);
    
    template std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZI>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> probs, float new_obj_thres);
    
    template std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> probs, float new_obj_thres);
}