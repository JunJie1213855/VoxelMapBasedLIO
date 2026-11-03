#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pv_lio {
// A downsampled history of registered scans, used only for visualization.
// The estimator keeps its own probabilistic plane map.
class DisplayMap {
 public:
  void set_resolution(double resolution) {
    if (!std::isfinite(resolution) || resolution <= 0.0)
      throw std::invalid_argument("Global display map resolution must be positive");
    if (resolution != resolution_) points_.clear();
    resolution_ = resolution;
  }

  template <typename PointT>
  void add(const pcl::PointCloud<PointT> &cloud) {
    for (const auto &p : cloud) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const double x = std::floor(p.x / resolution_);
      const double y = std::floor(p.y / resolution_);
      const double z = std::floor(p.z / resolution_);
      const double limit = 9223372036854775808.0;  // 2^63, exclusive upper bound
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          std::abs(x) >= limit || std::abs(y) >= limit || std::abs(z) >= limit) continue;
      const Key key{static_cast<int64_t>(x), static_cast<int64_t>(y), static_cast<int64_t>(z)};
      pcl::PointXYZI point;
      point.x = p.x; point.y = p.y; point.z = p.z;
      point.intensity = std::isfinite(p.intensity) ? p.intensity : 0.0f;
      points_[key] = point;
    }
  }

  pcl::PointCloud<pcl::PointXYZI> cloud() const {
    pcl::PointCloud<pcl::PointXYZI> result;
    result.reserve(points_.size());
    for (const auto &entry : points_) result.push_back(entry.second);
    result.is_dense = true;
    return result;
  }

  void coarsen_to(size_t max_points) {
    if (max_points < 8) throw std::invalid_argument("Display map limit must be at least 8 voxels");
    while (points_.size() > max_points) {
      auto previous = cloud();
      resolution_ *= 2.0;
      points_.clear();
      add(previous);
    }
  }

  bool empty() const { return points_.empty(); }
  size_t size() const { return points_.size(); }

 private:
  struct Key {
    int64_t x, y, z;
    bool operator==(const Key &other) const { return x == other.x && y == other.y && z == other.z; }
  };
  struct Hash {
    size_t operator()(const Key &key) const {
      size_t seed = std::hash<int64_t>{}(key.x);
      const auto combine = [&seed](int64_t value) {
        seed ^= std::hash<int64_t>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      };
      combine(key.y); combine(key.z);
      return seed;
    }
  };
  double resolution_ = 0.5;
  std::unordered_map<Key, pcl::PointXYZI, Hash> points_;
};
}  // namespace pv_lio
