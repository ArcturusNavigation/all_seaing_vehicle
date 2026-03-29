#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_HPP

#include <memory>
#include <vector>

#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/label_compat.hpp"
#include "all_seaing_perception/data_association/cost/mahalanobis.hpp"

namespace all_seaing_perception::data_association {

// Global greedy: repeatedly pick the overall minimum-cost (det, tracked) pair
// that is below threshold, marking both as used.
template<typename TrackedT, typename DetectedT>
AssociationResult greedy_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx)
{
    CostResult cost = compute_mahalanobis_cost(tracked, detected, ctx);
    const auto& p = cost.cost_matrix;

    AssociationResult result;
    result.match.assign(detected.size(), -1);

    float min_p = 0;
    while (min_p < ctx.threshold) {
        min_p = ctx.threshold;
        std::pair<int, int> best_match = {-1, -1};
        for (int i = 0; i < static_cast<int>(detected.size()); i++) {
            if (result.chosen_detected.count(i))
                continue;
            for (int tracked_id = 0; tracked_id < static_cast<int>(tracked.size()); tracked_id++) {
                if (result.chosen_tracked.count(tracked_id) ||
                    !label_compatible(detected[i]->label, tracked[tracked_id]->label, ctx.label_number_map))
                    continue;
                if (p[i][tracked_id] < min_p) {
                    best_match = {i, tracked_id};
                    min_p = p[i][tracked_id];
                }
            }
        }
        if (min_p < ctx.threshold) {
            result.match[best_match.first] = best_match.second;
            result.chosen_tracked.insert(best_match.second);
            result.chosen_detected.insert(best_match.first);
        }
    }
    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_HPP
