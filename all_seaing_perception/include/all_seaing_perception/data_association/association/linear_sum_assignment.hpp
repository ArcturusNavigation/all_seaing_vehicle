#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__LINEAR_SUM_ASSIGNMENT_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__LINEAR_SUM_ASSIGNMENT_HPP

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "all_seaing_perception/Hungarian.h"
#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/label_compat.hpp"
#include "all_seaing_perception/data_association/cost/mahalanobis.hpp"

namespace all_seaing_perception::data_association {

// Optimal assignment via the Hungarian algorithm. Augments the cost matrix with
// dummy rows/columns for non-assignment. Optionally uses sqrt of Mahalanobis
// distances instead of squared values.
template<typename TrackedT, typename DetectedT>
AssociationResult linear_sum_assignment_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx,
    bool use_sqrt = false)
{
    CostResult cost = compute_mahalanobis_cost(tracked, detected, ctx);
    const auto& p = cost.cost_matrix;

    AssociationResult result;
    int num_det = static_cast<int>(detected.size());
    int num_obj = static_cast<int>(tracked.size());
    result.match.assign(num_det, -1);

    const int INF = 1e9;
    Eigen::MatrixXf cost_matrix = Eigen::MatrixXf::Zero(num_det + num_obj, num_obj + num_det);
    float max_val = 0;
    for (int i = 0; i < num_det; i++) {
        for (int tracked_id = 0; tracked_id < num_obj; tracked_id++) {
            if (!label_compatible(detected[i]->label, tracked[tracked_id]->label, ctx.label_number_map) ||
                p[i][tracked_id] >= ctx.threshold) {
                cost_matrix(i, tracked_id) = INF;
            } else {
                cost_matrix(i, tracked_id) = use_sqrt ? std::sqrt(p[i][tracked_id]) : p[i][tracked_id];
                max_val = std::max(max_val, cost_matrix(i, tracked_id));
            }
        }
    }
    cost_matrix.block(num_det, 0, num_obj, num_obj).fill(max_val + 1);
    cost_matrix.block(0, num_obj, num_det + num_obj, num_det).fill(max_val + 1);

    // Convert to vector of vectors for Hungarian algorithm
    std::vector<std::vector<double>> cost_matrix_vec;
    for (int i = 0; i < num_det + num_obj; i++) {
        cost_matrix_vec.push_back(std::vector<double>());
        for (int j = 0; j < num_obj + num_det; j++) {
            cost_matrix_vec[i].push_back(cost_matrix(i, j));
        }
    }

    HungarianAlgorithm HungAlgo;
    std::vector<int> assignment;
    HungAlgo.Solve(cost_matrix_vec, assignment);

    for (int i = 0; i < num_det; i++) {
        if (assignment[i] < num_obj && cost_matrix(i, assignment[i]) != INF) {
            result.match[i] = assignment[i];
        }
    }

    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__LINEAR_SUM_ASSIGNMENT_HPP
