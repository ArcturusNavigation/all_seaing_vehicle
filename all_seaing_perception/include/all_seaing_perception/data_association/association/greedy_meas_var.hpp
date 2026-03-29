#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_MEAS_VAR_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_MEAS_VAR_HPP

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/label_compat.hpp"
#include "all_seaing_perception/data_association/cost/mahalanobis.hpp"

namespace all_seaing_perception::data_association {

// Greedy association that processes (detection, tracked) pairs in order of
// increasing measurement variance (Psi trace for that pair).
template<typename TrackedT, typename DetectedT>
AssociationResult greedy_meas_var_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx)
{
    CostResult cost = compute_mahalanobis_cost(tracked, detected, ctx);
    const auto& p = cost.cost_matrix;
    const auto& v_meas = cost.meas_var;

    AssociationResult result;
    result.match.assign(detected.size(), -1);

    std::vector<std::pair<int, int>> pair_ind;
    for (int i = 0; i < static_cast<int>(detected.size()); i++) {
        for (int j = 0; j < static_cast<int>(tracked.size()); j++) {
            pair_ind.push_back({i, j});
        }
    }
    std::sort(pair_ind.begin(), pair_ind.end(),
              [&v_meas](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                  return v_meas[a.first][a.second] < v_meas[b.first][b.second];
              });

    for (const auto& [i, tracked_id] : pair_ind) {
        if (result.chosen_detected.count(i))
            continue;
        if (result.chosen_tracked.count(tracked_id) ||
            !label_compatible(detected[i]->label, tracked[tracked_id]->label, ctx.label_number_map))
            continue;
        if (p[i][tracked_id] < ctx.threshold) {
            result.match[i] = tracked_id;
            result.chosen_detected.insert(i);
            result.chosen_tracked.insert(tracked_id);
        }
    }
    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_MEAS_VAR_HPP
