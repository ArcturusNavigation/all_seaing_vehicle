#ifndef ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__LABEL_COMPAT_HPP
#define ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__LABEL_COMPAT_HPP

#include <map>

namespace all_seaing_perception::data_association {

// Returns true if a detected label is compatible with a tracked label.
// Handles normal label matching and banner label-number grouping.
inline bool label_compatible(int det_label, int tracked_label,
                             const std::map<int, int>* label_number_map = nullptr) {
    if (det_label == -1) {
        return true;
    }
    if (label_number_map != nullptr &&
        label_number_map->count(det_label) &&
        label_number_map->count(tracked_label) &&
        label_number_map->at(det_label) == label_number_map->at(tracked_label)) {
        return true;
    }
    return tracked_label == det_label;
}

}  // namespace all_seaing_perception::data_association

#endif  // ALL_SEAING_PERCEPTION__DATA_ASSOCIATION__LABEL_COMPAT_HPP
