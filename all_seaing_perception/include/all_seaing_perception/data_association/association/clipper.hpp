#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__CLIPPER_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__CLIPPER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cstdio>

#include <Eigen/Dense>

#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/label_compat.hpp"

#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__COST__MAHALANOBIS_HPP
#include "all_seaing_perception/data_association/cost/mahalanobis.hpp"
#endif

namespace all_seaing_perception::data_association {
// TODO: A cost function that also favors "discrete" u

template<typename TrackedT, typename DetectedT>
AssociationResult clipper_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx)
{
    using FloatT = double;
    CostResult cost = compute_mahalanobis_cost(tracked, detected, ctx);
    const auto& p = cost.cost_matrix;

    struct Candidate {
        int tracked_id;
        int detected_id;
    };

    // TODO: I stole this from ROMAN, test to see if other cost functions work ok
    // dist cost on diagonal just in case there aren't many options
    const FloatT ROMAN_COST_STDDEV = 1.0;
    const FloatT ROMAN_DIST_THRESH = 2.0;
    const FloatT DIST_CONTRIBUTE = 0.5;
    const FloatT DIST_THRESH = 0.5;
    auto roman_cost = [&](const Candidate& x, const Candidate& y) -> FloatT {
        if (x.tracked_id == y.tracked_id || x.detected_id == y.detected_id){
            FloatT dist = p[x.detected_id][x.tracked_id];
            return (x.tracked_id == y.tracked_id && x.detected_id == y.detected_id && dist < ctx.clipper_cull_threshold) * (1.0 + DIST_CONTRIBUTE / (1.0 + dist));
        }

        Eigen::Vector2f x_track = tracked[x.tracked_id]->mean_pred.template head<2>();
        Eigen::Vector2f y_track = tracked[y.tracked_id]->mean_pred.template head<2>();

        // Convert detected (range, bearing) to local (x, y) for distance comparison
        auto rb_to_xy = [&](const auto& det) -> Eigen::Vector2f {
            float r = det->mean_pred[0];
            float b = det->mean_pred[1];
            return Eigen::Vector2f(r * std::cos(b), r * std::sin(b));
        };
        Eigen::Vector2f x_detect = rb_to_xy(detected[x.detected_id]);
        Eigen::Vector2f y_detect = rb_to_xy(detected[y.detected_id]);

        FloatT d_track = (x_track - y_track).template cast<FloatT>().norm();
        FloatT d_detect = (x_detect - y_detect).template cast<FloatT>().norm();
        FloatT delta_abs = std::abs(d_track - d_detect);
        printf(" (%f) ", delta_abs);

        if (delta_abs > ROMAN_DIST_THRESH)
            return 0.0;
        return std::exp(-delta_abs * delta_abs / (2.0 * ROMAN_COST_STDDEV * ROMAN_COST_STDDEV));
    };

    // Build candidate associations from label-compatible pairs
    std::vector<Candidate> candidates;
    for (int i = 0; i < static_cast<int>(tracked.size()); ++i) {
        for (int j = 0; j < static_cast<int>(detected.size()); ++j) {
            if (label_compatible(detected[j]->label, tracked[i]->label, ctx.label_number_map)) {
                candidates.push_back(Candidate{i, j});
            }
        }
    }

    int n = static_cast<int>(candidates.size());

    AssociationResult result;
    result.match.assign(detected.size(), -1);
    if (n == 0) return result;

    Eigen::MatrixXd M(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = i; j < n; ++j) {
            FloatT val = roman_cost(candidates[i], candidates[j]);
            M(i, j) = val;
            M(j, i) = val;
        }
    }

    const FloatT d_grow = 1.02;
    const FloatT d_min = 0.01;
    const FloatT alpha_min = 1e-5;
    const FloatT alpha_decay = 0.9;
    const FloatT u_converge_thresh = 0.001;
    const FloatT ARMIJO_COEF = 0.001;
    const FloatT MIN_ORTH_GRAD_MAG = 1e-6;
    const FloatT SELECT_MAX_RATIO = 0.5; // what fraction of max value you have to be to be selected

    Eigen::VectorXd u = Eigen::VectorXd::Constant(n, 1.0 / std::sqrt(static_cast<FloatT>(n)));

    // questionable d initialization, don't question it
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(M);
    const auto& eigenvalues = eigensolver.eigenvalues();
    FloatT lambda_min = eigenvalues(0);
    FloatT lambda_max = eigenvalues(n - 1);
    FloatT d = std::max(d_min, -lambda_min);
    FloatT d_max = std::max(d, lambda_max);

    Eigen::MatrixXd M_d = M;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (M_d(i, j) <= 0) M_d(i, j) -= d;

    auto evaluate = [&](const Eigen::VectorXd& u_t) -> FloatT {
        return u_t.dot(M_d * u_t);
    };

    while (d < d_max) {
        FloatT eval_u = evaluate(u);
        while (true) {
            Eigen::VectorXd raw_grad = 2.0 * M_d * u;

            Eigen::VectorXd orth_grad = raw_grad - u.dot(raw_grad) * u;
            FloatT norm_orth = orth_grad.norm();

            if (norm_orth < MIN_ORTH_GRAD_MAG) break;

            FloatT alpha = std::numeric_limits<FloatT>::max();
            for (int i = 0; i < n; ++i) {
                if (orth_grad(i) < 0 && u(i) > 0) {
                    alpha = std::min(alpha, -u(i) / orth_grad(i));
                }
            }

            Eigen::VectorXd new_u(n);
            bool found_step = false;
            while (alpha >= alpha_min) {
                new_u = u + alpha * orth_grad;
                FloatT e = evaluate(new_u);
                if (e - eval_u >= norm_orth * alpha * ARMIJO_COEF) {
                    found_step = true;
                    break;
                }
                alpha *= alpha_decay;
            }

            if (!found_step) break;

            new_u = new_u.cwiseMax(0.0);
            FloatT mag = new_u.norm();
            if (mag < 1e-12) break;  // all entries zeroed out, no valid associations
            new_u /= mag;

            if ((new_u - u).norm() < u_converge_thresh) {
                u = new_u;
                break;
            }
            u = new_u;
            eval_u = evaluate(u);
        }

        FloatT d_new = d * d_grow;
        FloatT delta_d = d_new - d;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (M(i, j) <= 0) M_d(i, j) -= delta_d;
        d = d_new;
    }

    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&u](int a, int b) { return u(a) > u(b); });

    FloatT u_max = u(indices[0]);
    FloatT select_thresh = SELECT_MAX_RATIO * u_max;

    std::unordered_set<int> used_tracked, used_detected;
    for (int idx : indices) {
        if (u(idx) < select_thresh) break;
        const Candidate& c = candidates[idx];
        if (used_tracked.count(c.tracked_id) || used_detected.count(c.detected_id))
            continue;
        result.match[c.detected_id] = c.tracked_id;
        result.chosen_tracked.insert(c.tracked_id);
        result.chosen_detected.insert(c.detected_id);
        used_tracked.insert(c.tracked_id);
        used_detected.insert(c.detected_id);
    }

    {
        auto rb_to_xy = [&](const auto& det) -> Eigen::Vector2f {
            float r = det->mean_pred[0];
            float b = ctx.robot_state[2] + det->mean_pred[1];
            return Eigen::Vector2f(ctx.robot_state[0] + r * std::cos(b), ctx.robot_state[1] + r * std::sin(b));
        };
        printf("Data: %f\n",ctx.threshold);
        printf("Tracked:\n");
        int i=0;
        for(auto&tobj:tracked){
            int offset = ctx.state_start_offset + 2 * i;
            Eigen::Vector2f pos = Eigen::Vector2f((*ctx.full_state)(offset), (*ctx.full_state)(offset+1));
            printf("(%f, %f) %d\n",pos[0],pos[1],tobj->label);
            i++;
        }
        printf("Detected:\n");
        for(auto&dobj:detected){
            Eigen::Vector2f pos = rb_to_xy(dobj);
            printf("(%f, %f) %d\n",pos[0],pos[1],dobj->label);
        }
        printf("Match:\n");
        for(int i=0;i<result.match.size();++i){
            printf("det %d track %d ",i,result.match[i]);
        }printf("\n");
        printf("Associations:\n");
        for(auto&x:candidates){
            printf("det %d track %d\n", x.detected_id, x.tracked_id);
        }
        printf("\nMat:\n");
        for(int a=0;a<n;++a){for(int b=0;b<n;++b)printf("%f ", roman_cost(candidates[a], candidates[b]));printf("\n");}
        printf("\nMabh:\n");
        for(auto&a:p){for(auto&b:a)printf("%f ",b);printf("\n");}
    }

    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__CLIPPER_HPP
