#pragma once
#include "display_map.hpp"
#include "latest_task_worker.hpp"
#include "voxel_map_util.hpp"
#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace pv_lio {
// Only value copies cross the thread boundary; the worker never touches the
// estimator's octrees, filter state, or mutable input buffers.
class AsyncMapPublisher {
 public:
  struct Snapshot {
    PointCloudXYZI::ConstPtr world_cloud;
    std::vector<Plane> planes;
    builtin_interfaces::msg::Time stamp;
  };
  AsyncMapPublisher(bool clouds, bool planes, double resolution, double period, size_t max_points, size_t max_planes,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plane_pub)
      : clouds_(clouds), planes_(planes), period_(period), max_points_(max_points), max_planes_(max_planes), cloud_pub_(std::move(cloud_pub)),
        plane_pub_(std::move(plane_pub)), worker_([this](Snapshot snapshot) { publish(std::move(snapshot)); }, [this] { repeat_cached(); }) {
    display_map_.set_resolution(resolution);
  }
  bool due() const {
    return (clouds_ || planes_) && worker_.idle() &&
        std::chrono::steady_clock::now() >= next_snapshot_;
  }
  bool submit(Snapshot snapshot) {
    if (!worker_.submit(std::move(snapshot))) return false;
    next_snapshot_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(period_));
    return true;
  }
  void stop() { worker_.stop(); }
 private:
  void publish(Snapshot snapshot) {
    try {
      if (!rclcpp::ok()) return;
      if (clouds_ && snapshot.world_cloud) {
        display_map_.add(*snapshot.world_cloud);
        display_map_.coarsen_to(max_points_);
        sensor_msgs::msg::PointCloud2 message;
        const auto cloud = display_map_.cloud();
        pcl::toROSMsg(cloud, message);
        message.header.frame_id = "camera_init";
        message.header.stamp = snapshot.stamp;
        latest_cloud_.reset(new sensor_msgs::msg::PointCloud2(std::move(message)));
        cloud_pub_->publish(*latest_cloud_);
      }
      if (planes_ && rclcpp::ok()) {
        visualization_msgs::msg::MarkerArray message;
        message.markers.reserve(snapshot.planes.size() + 1);
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = "camera_init";
        clear.header.stamp = snapshot.stamp;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        message.markers.push_back(clear);
        std::vector<const Plane *> visible;
        visible.reserve(snapshot.planes.size());
        for (const auto &plane : snapshot.planes) if (plane.is_plane) visible.push_back(&plane);
        std::sort(visible.begin(), visible.end(), [](const Plane *a, const Plane *b) { return a->id < b->id; });
        const size_t count = std::min(max_planes_, visible.size());
        for (size_t i = 0; i < count; ++i) {
          const auto &plane = *visible[i * visible.size() / count];
          double trace = plane.plane_cov.block<3, 3>(0, 0).diagonal().sum();
          trace = std::pow(std::max(0.0, std::min(0.25, trace)) / 0.25, 0.2);
          uint8_t r, g, b;
          mapJet(trace, 0.0, 1.0, r, g, b);
          pubSinglePlane(message, "plane", plane, 0.8f, Eigen::Vector3d(r / 256.0, g / 256.0, b / 256.0));
          message.markers.back().header.stamp = snapshot.stamp;
        }
        latest_planes_.reset(new visualization_msgs::msg::MarkerArray(std::move(message)));
        plane_pub_->publish(*latest_planes_);
      }
      last_publication_ = std::chrono::steady_clock::now();
    } catch (const std::exception &error) {
      if (rclcpp::ok()) RCLCPP_WARN(rclcpp::get_logger("pv_lio_visualization"),
          "Skipped visualization update: %s", error.what());
    }
  }
  void repeat_cached() {
    if (!rclcpp::ok() || std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_publication_).count() < period_) return;
    // Best-effort historical delivery varies by DDS implementation. Repeating
    // cached values also serves RViz after sensor input has stopped.
    if (latest_cloud_ && cloud_pub_->get_subscription_count() > 0) cloud_pub_->publish(*latest_cloud_);
    if (latest_planes_ && plane_pub_->get_subscription_count() > 0) plane_pub_->publish(*latest_planes_);
    last_publication_ = std::chrono::steady_clock::now();
  }
  bool clouds_, planes_;
  double period_;
  size_t max_points_, max_planes_;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> latest_cloud_;
  std::unique_ptr<visualization_msgs::msg::MarkerArray> latest_planes_;
  std::chrono::steady_clock::time_point last_publication_{};
  DisplayMap display_map_;  // owned exclusively by the worker
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plane_pub_;
  std::chrono::steady_clock::time_point next_snapshot_{};  // producer thread only
  LatestTaskWorker<Snapshot> worker_;
};
class AsyncScanPublisher {
 public:
  struct Snapshot {
    PointCloudXYZI::ConstPtr cloud;
    M3D rotation, extrinsic_rotation;
    V3D position, extrinsic_translation;
    builtin_interfaces::msg::Time stamp;
  };
  AsyncScanPublisher(bool enabled, bool body_enabled,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_pub,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_pub)
      : enabled_(enabled), body_enabled_(body_enabled), world_pub_(std::move(world_pub)),
        body_pub_(std::move(body_pub)), worker_([this](Snapshot snapshot) { publish(std::move(snapshot)); }) {}
  bool ready() const {
    return enabled_ && worker_.idle() && (world_pub_->get_subscription_count() > 0 ||
        (body_enabled_ && body_pub_->get_subscription_count() > 0));
  }
  void submit(Snapshot snapshot) { worker_.submit(std::move(snapshot)); }
  void stop() { worker_.stop(); }
 private:
  void publish(Snapshot snapshot) {
    try {
      if (!rclcpp::ok()) return;
      const bool world_needed = world_pub_->get_subscription_count() > 0;
      const bool body_needed = body_enabled_ && body_pub_->get_subscription_count() > 0;
      PointCloudXYZI world, body;
      if (world_needed) world.reserve(snapshot.cloud->size());
      if (body_needed) body.reserve(snapshot.cloud->size());
      for (const auto &p : *snapshot.cloud) {
        const V3D in_body = snapshot.extrinsic_rotation * V3D(p.x, p.y, p.z) + snapshot.extrinsic_translation;
        if (body_needed) {
          PointType point = p;
          point.x = in_body.x(); point.y = in_body.y(); point.z = in_body.z();
          body.push_back(point);
        }
        if (world_needed && p.intensity >= 5.0f) {
          const V3D in_world = snapshot.rotation * in_body + snapshot.position;
          PointType point = p;
          point.x = in_world.x(); point.y = in_world.y(); point.z = in_world.z();
          world.push_back(point);
        }
      }
      if (world_needed && rclcpp::ok()) {
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(world, message);
        message.header.frame_id = "camera_init"; message.header.stamp = snapshot.stamp;
        world_pub_->publish(message);
      }
      if (body_needed && rclcpp::ok()) {
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(body, message);
        message.header.frame_id = "body"; message.header.stamp = snapshot.stamp;
        body_pub_->publish(message);
      }
    } catch (const std::exception &error) {
      if (rclcpp::ok()) RCLCPP_WARN(rclcpp::get_logger("pv_lio_visualization"), "Skipped scan display: %s", error.what());
    }
  }
  bool enabled_, body_enabled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_pub_, body_pub_;
  LatestTaskWorker<Snapshot> worker_;
};
}  // namespace pv_lio
