#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>

namespace voxelmap_input {
// PointCloud2 fields may be unaligned, organized with row padding, or big endian.
// Read numeric fields without assuming a compiler-specific point-struct layout.
class CloudFields {
 public:
  explicit CloudFields(const sensor_msgs::msg::PointCloud2 &message) : message_(message) {
    const uint64_t row_bytes = uint64_t(message.width) * message.point_step;
    const uint64_t bytes = message.height ? uint64_t(message.height-1)*message.row_step + row_bytes : 0;
    if (!message.point_step || message.row_step < row_bytes || bytes > message.data.size())
      throw std::invalid_argument("truncated or invalid PointCloud2 dimensions/steps");
    const uint16_t one = 1;
    swap_ = message.is_bigendian != (*reinterpret_cast<const uint8_t *>(&one) == 0);
  }
  const sensor_msgs::msg::PointField *find(const std::string &name) const {
    for (const auto &field : message_.fields) if (field.name == name) {
      const size_t bytes = size(field.datatype);
      if (!bytes || field.count != 1 || uint64_t(field.offset)+bytes > message_.point_step)
        throw std::invalid_argument("invalid PointCloud2 field: " + name);
      return &field;
    }
    return nullptr;
  }
  const sensor_msgs::msg::PointField &require(const std::string &name) const {
    auto field = find(name);
    if (!field) throw std::invalid_argument("missing PointCloud2 field: " + name);
    return *field;
  }
  double read(size_t index, const sensor_msgs::msg::PointField &field) const {
    const size_t offset = (index / message_.width)*message_.row_step +
                          (index % message_.width)*message_.point_step + field.offset;
    const auto *source = message_.data.data()+offset;
    using F = sensor_msgs::msg::PointField;
    switch (field.datatype) {
      case F::INT8: return value<int8_t>(source);
      case F::UINT8: return value<uint8_t>(source);
      case F::INT16: return value<int16_t>(source);
      case F::UINT16: return value<uint16_t>(source);
      case F::INT32: return value<int32_t>(source);
      case F::UINT32: return value<uint32_t>(source);
      case F::FLOAT32: return value<float>(source);
      case F::FLOAT64: return value<double>(source);
      default: throw std::invalid_argument("unsupported PointCloud2 datatype");
    }
  }
 private:
  static size_t size(uint8_t type) {
    switch(type) {
      case 1: case 2: return 1;
      case 3: case 4: return 2;
      case 5: case 6: case 7: return 4;
      case 8: return 8;
      default: return 0;
    }
  }
  template <typename T> T value(const uint8_t *source) const {
    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, source, sizeof(T));
    if (swap_) std::reverse(bytes, bytes+sizeof(T));
    T result;
    std::memcpy(&result, bytes, sizeof(T));
    return result;
  }
  const sensor_msgs::msg::PointCloud2 &message_;
  bool swap_;
};
} // namespace voxelmap_input
