#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_INDIV_VAR_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_INDIV_VAR_HPP

#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>

#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/label_compat.hpp"
#include "all_seaing_perception/data_association/cost/mahalanobis.hpp"

namespace all_seaing_perception::data_association {

// Greedy association that processes tracked objects in order of increasing
// individual variance (trace of their covariance block).
template<typename TrackedT, typename DetectedT>
AssociationResult greedy_indiv_var_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx)
{
    CostResult cost = compute_mahalanobis_cost(tracked, detected, ctx);
    const auto& p = cost.cost_matrix;
    const auto& v_indiv = cost.indiv_var;

    AssociationResult result;
    result.match.assign(detected.size(), -1);

    std::vector<int> tracked_indices(tracked.size());
    std::iota(tracked_indices.begin(), tracked_indices.end(), 0);
    std::sort(tracked_indices.begin(), tracked_indices.end(),
              [&v_indiv](int a, int b) { return v_indiv[a] < v_indiv[b]; });

    for (int tracked_id : tracked_indices) {
        float min_p = ctx.threshold;
        int best_match = -1;
        for (int i = 0; i < static_cast<int>(detected.size()); i++) {
            if (result.chosen_detected.count(i) ||
                !label_compatible(detected[i]->label, tracked[tracked_id]->label, ctx.label_number_map))
                continue;
            if (p[i][tracked_id] < min_p) {
                best_match = i;
                min_p = p[i][tracked_id];
            }
        }
        if (min_p < ctx.threshold) {
            result.match[best_match] = tracked_id;
            result.chosen_tracked.insert(tracked_id);
            result.chosen_detected.insert(best_match);
        }
    }
    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_INDIV_VAR_HPP
