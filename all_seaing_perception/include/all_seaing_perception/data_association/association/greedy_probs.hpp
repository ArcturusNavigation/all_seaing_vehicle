#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_PROBS_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_PROBS_HPP

#include <memory>
#include <tuple>
#include <vector>

#include "all_seaing_perception/data_association/types.hpp"
#include "all_seaing_perception/data_association/association/greedy.hpp"

namespace all_seaing_perception::data_association {

// Greedy association that also computes a particle weight from detection
// probabilities. Returns (weight, result) where weight is the product of
// probabilities for each detection's correspondence.
template<typename TrackedT, typename DetectedT>
std::tuple<float, AssociationResult> greedy_probs_associate(
    const std::vector<std::shared_ptr<TrackedT>>& tracked,
    const std::vector<std::shared_ptr<DetectedT>>& detected,
    const std::vector<std::vector<float>>& probs,
    const AssociationContext& ctx)
{
    AssociationResult result = greedy_associate(tracked, detected, ctx);

    // probs' last element per detection has the probability that an object is newly detected
    float weight = 1;
    for (size_t i = 0; i < detected.size(); i++) {
        if (result.match[i] == -1) {
            weight *= probs[i].back();
        } else {
            weight *= probs[i][result.match[i]];
        }
    }
    return {weight, result};
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__ASSOCIATION__GREEDY_PROBS_HPP
