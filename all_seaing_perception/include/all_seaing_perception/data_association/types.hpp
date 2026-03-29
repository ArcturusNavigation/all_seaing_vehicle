#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__TYPES_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__TYPES_HPP

#include <map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

namespace all_seaing_perception::data_association {

struct AssociationResult {
    std::vector<int> match;                    // match[det_i] = tracked_id or -1
    std::unordered_set<int> chosen_detected;
    std::unordered_set<int> chosen_tracked;
};

struct AssociationContext {
    Eigen::Vector3f robot_state;        // [x, y, theta]
    Eigen::MatrixXf meas_noise_cov;     // Q matrix (2x2 or 3x3)
    float threshold;                     // Mahalanobis distance threshold

    // For EKF SLAM with joint state (m_track_robot=true). nullptr if not used.
    const Eigen::VectorXf* full_state = nullptr;
    const Eigen::MatrixXf* full_cov = nullptr;
    int mat_size = 0;
    int state_start_offset = 3;          // Index in full_state where tracked elements begin

    // For banner label grouping. nullptr if not used.
    const std::map<int, int>* label_number_map = nullptr;
};

struct CostResult {
    std::vector<std::vector<float>> cost_matrix;  // [num_det][num_tracked]
    std::vector<float> indiv_var;                  // [num_tracked]
    std::vector<std::vector<float>> meas_var;      // [num_det][num_tracked]
};

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__TYPES_HPP
