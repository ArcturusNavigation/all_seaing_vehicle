#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__INDIV_GREEDY_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__INDIV_GREEDY_HPP

#include <memory>
#include <vector>

#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/label_compat.hpp"
#include "all_seaing_perception/data_association/cost/mahalanobis.hpp"

namespace all_seaing_perception::data_association {

// Per-detection greedy: for each detection independently, pick the closest
// tracked object below threshold. Does NOT enforce exclusive assignment
// (multiple detections may map to the same tracked object).
template<typename TrackedT, typename DetectedT>
AssociationResult indiv_greedy_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const AssociationContext& ctx)
{
    CostResult cost = compute_mahalanobis_cost(tracked, detected, ctx);
    const auto& p = cost.cost_matrix;

    AssociationResult result;
    result.match.assign(detected.size(), -1);

    for (int i = 0; i < static_cast<int>(detected.size()); i++) {
        float min_p = ctx.threshold;
        int best_match = -1;
        for (int tracked_id = 0; tracked_id < static_cast<int>(tracked.size()); tracked_id++) {
            if (!label_compatible(detected[i]->label, tracked[tracked_id]->label, ctx.label_number_map))
                continue;
            if (p[i][tracked_id] < min_p) {
                best_match = tracked_id;
                min_p = p[i][tracked_id];
            }
        }
        if (min_p < ctx.threshold) {
            result.match[i] = best_match;
            result.chosen_tracked.insert(best_match);
            result.chosen_detected.insert(i);
        }
    }
    return result;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__INDIV_GREEDY_HPP
