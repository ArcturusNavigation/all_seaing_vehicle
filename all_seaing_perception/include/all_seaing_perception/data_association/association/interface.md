# Association Template Interface

The association functions are templated on `TrackedT` and `DetectedT`. These are
the struct types stored behind `std::shared_ptr` in the tracked/detected vectors.

## DetectedT

Must have:
```cpp
int label;              // -1 means "unknown/unlabeled"
Eigen::Vector2f mean_pred;  // measurement: (range, bearing) for 2D obstacles
// or
Eigen::Vector3f mean_pred;  // measurement: (range, bearing, phi) for 3D banners
```

The caller is responsible for populating `mean_pred` with the range-bearing
measurement before calling the association function. The dimension of `mean_pred`
(2 vs 3) determines 2D vs 3D cost computation.

## TrackedT

Must have:
```cpp
int label;
Eigen::Vector2f mean_pred;  // predicted global position (x, y) for 2D
Eigen::Matrix2f cov;        // covariance of position estimate
// or
Eigen::Vector3f mean_pred;  // predicted global position (x, y, theta) for 3D
Eigen::Matrix3f cov;        // covariance of position estimate
```

`mean_pred` and `cov` are only accessed inside `compute_mahalanobis_cost` in the
non-SLAM branch (when `ctx.full_state == nullptr`). In SLAM mode, positions and
covariances come from `ctx.full_state` / `ctx.full_cov` instead.

## Concrete types that satisfy these interfaces

- `all_seaing_perception::ObjectCloud<PointT>` — 2D obstacles (both tracked and detected)
- `all_seaing_perception::Banner` — 3D banners (both tracked and detected)
