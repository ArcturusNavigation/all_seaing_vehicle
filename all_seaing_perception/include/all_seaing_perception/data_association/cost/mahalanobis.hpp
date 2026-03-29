#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__COST__MAHALANOBIS_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__COST__MAHALANOBIS_HPP

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "all_seaing_perception/object_tracking_shared.hpp"
#include "all_seaing_perception/data_association/types.hpp"

namespace all_seaing_perception::data_association {

// Compute the squared Mahalanobis distance cost matrix between detected and tracked elements.
// Reads detected[i]->mean_pred as the measurement (range, bearing[, phi]).
// Handles both 2D (obstacles: range, bearing) and 3D (banners: range, bearing, phi) cases
// by checking the measurement vector dimension at runtime.
// Branches on ctx.full_state != nullptr for EKF SLAM joint-state mode.
template<typename TrackedT, typename DetectedT>
CostResult compute_mahalanobis_cost(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx)
{
    CostResult result;
    int num_tracked = static_cast<int>(tracked.size());
    int num_det = static_cast<int>(detected.size());
    result.cost_matrix.resize(num_det);
    result.meas_var.resize(num_det);
    result.indiv_var.resize(num_tracked, 0.0f);

    // Compute per-tracked individual variance
    for (int tracked_id = 0; tracked_id < num_tracked; tracked_id++) {
        int dim = tracked[tracked_id]->mean_pred.size();
        if (ctx.full_state != nullptr) {
            int offset = ctx.state_start_offset + dim * tracked_id;
            float var_sum = 0;
            for (int d = 0; d < dim; d++) {
                var_sum += (*ctx.full_cov)(offset + d, offset + d);
            }
            result.indiv_var[tracked_id] = var_sum;
        } else {
            float var_sum = 0;
            for (int d = 0; d < dim; d++) {
                var_sum += tracked[tracked_id]->cov(d, d);
            }
            result.indiv_var[tracked_id] = var_sum;
        }
    }

    // Compute cost matrix
    for (int i = 0; i < num_det; i++) {
        result.cost_matrix[i].resize(num_tracked);
        result.meas_var[i].resize(num_tracked);
        // Read measurement from detected object's mean_pred
        Eigen::VectorXf z_actual_raw = detected[i]->mean_pred.template cast<float>();
        int dim = static_cast<int>(z_actual_raw.size());

        for (int tracked_id = 0; tracked_id < num_tracked; tracked_id++) {
            Eigen::VectorXf z_pred;
            Eigen::MatrixXf Psi;

            if (ctx.full_state != nullptr) {
                int offset = ctx.state_start_offset + dim * tracked_id;
                float d_x = (*ctx.full_state)(offset) - (*ctx.full_state)(0);
                float d_y = (*ctx.full_state)(offset + 1) - (*ctx.full_state)(1);
                float q = d_x * d_x + d_y * d_y;

                if (dim == 2) {
                    z_pred = Eigen::Vector2f(
                        std::sqrt(q),
                        all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - (*ctx.full_state)(2)));

                    Eigen::MatrixXf F = Eigen::MatrixXf::Zero(5, ctx.mat_size);
                    F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
                    F.block(3, offset, 2, 2) = Eigen::Matrix2f::Identity();

                    Eigen::Matrix<float, 2, 5> h{
                        {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x, std::sqrt(q) * d_y},
                        {d_y, -d_x, -q, -d_y, d_x},
                    };
                    Eigen::MatrixXf H = h * F / q;
                    Psi = H * (*ctx.full_cov) * H.transpose() + ctx.meas_noise_cov;
                } else {
                    // dim == 3 (banners)
                    float d_theta = (*ctx.full_state)(offset + 2) - (*ctx.full_state)(2);
                    z_pred = Eigen::Vector3f(
                        std::sqrt(q),
                        all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - (*ctx.full_state)(2)),
                        all_seaing_perception::mod_2pi(d_theta));

                    Eigen::MatrixXf F = Eigen::MatrixXf::Zero(6, ctx.mat_size);
                    F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
                    F.block(3, offset, 3, 3) = Eigen::Matrix3f::Identity();

                    Eigen::Matrix<float, 3, 6> h{
                        {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x, std::sqrt(q) * d_y, 0},
                        {d_y, -d_x, -q, -d_y, d_x, 0},
                        {0, 0, -q, 0, 0, q},
                    };
                    Eigen::MatrixXf H = h * F / q;
                    Psi = H * (*ctx.full_cov) * H.transpose() + ctx.meas_noise_cov;
                }
            } else {
                float d_x = tracked[tracked_id]->mean_pred[0] - ctx.robot_state[0];
                float d_y = tracked[tracked_id]->mean_pred[1] - ctx.robot_state[1];
                float q = d_x * d_x + d_y * d_y;

                if (dim == 2) {
                    z_pred = Eigen::Vector2f(
                        std::sqrt(q),
                        all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - ctx.robot_state[2]));

                    Eigen::Matrix<float, 2, 2> h{
                        {std::sqrt(q) * d_x, std::sqrt(q) * d_y},
                        {-d_y, d_x},
                    };
                    Eigen::MatrixXf H = h / q;
                    Psi = H * tracked[tracked_id]->cov * H.transpose() + ctx.meas_noise_cov;
                } else {
                    // dim == 3 (banners)
                    float d_theta = tracked[tracked_id]->mean_pred[2] - ctx.robot_state[2];
                    z_pred = Eigen::Vector3f(
                        std::sqrt(q),
                        all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - ctx.robot_state[2]),
                        all_seaing_perception::mod_2pi(d_theta));

                    Eigen::Matrix<float, 3, 3> h{
                        {std::sqrt(q) * d_x, std::sqrt(q) * d_y, 0},
                        {-d_y, d_x, 0},
                        {0, 0, q},
                    };
                    Eigen::MatrixXf H = h / q;
                    Psi = H * tracked[tracked_id]->cov * H.transpose() + ctx.meas_noise_cov;
                }
            }

            // Adjust angular components to handle wrapping
            Eigen::VectorXf z_adj = z_actual_raw;
            z_adj(1) = z_pred(1) + all_seaing_perception::angle_to_pi_range(z_adj(1) - z_pred(1));
            if (dim == 3) {
                z_adj(2) = z_pred(2) + all_seaing_perception::angle_to_pi_range(z_adj(2) - z_pred(2));
            }

            result.cost_matrix[i][tracked_id] =
                (z_adj - z_pred).transpose() * Psi.inverse() * (z_adj - z_pred);
            result.meas_var[i][tracked_id] = Psi.trace();
        }
    }

    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__COST__MAHALANOBIS_HPP
