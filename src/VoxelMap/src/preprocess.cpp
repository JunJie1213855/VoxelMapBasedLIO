#include "preprocess.h"
#include "pointcloud_fields.hpp"
#include <limits>

#include <algorithm>
#include <cmath>

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1) {
  N_SCANS = 6;
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  given_offset_time = false;
  scan_start_time = rclcpp::Time(msg->header.stamp).seconds();
  scan_end_time = scan_start_time;
  switch (lidar_type) {
  case L515:
    l515_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case ROBOSENSE_AIRY:
    robosense_airy_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  if (!pl_surf.empty()) scan_end_time = std::max(scan_end_time,
      scan_start_time + pl_surf.back().curvature / 1000.0);
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num;
  std::vector<bool> is_valid_pt(plsize, false);

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++) {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  for (uint i = 1; i < plsize; i++) {
    if ((msg->points[i].line < N_SCANS) &&
        ((msg->points[i].tag & 0x30) == 0x10 ||
         (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (i % point_filter_num == 0) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature =
            msg->points[i].offset_time /
            float(1000000); // use curvature as time of each laser points

        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
            (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) &&
                (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                     pl_full[i].z + pl_full[i].z >
                 blind * blind)) {
          is_valid_pt[i] = true;
        }
      }
    }
  }

  for (uint i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      pl_surf.points.push_back(pl_full[i]);
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
  for (int i = 0; i < pl_orig.points.size(); i++) {
    if (i % point_filter_num != 0)
      continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < blind)
      continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
    if (yaw_angle >= 180.0)
      yaw_angle -= 360.0;
    if (yaw_angle <= -180.0)
      yaw_angle += 360.0;

    added_pt.curvature = pl_orig.points[i].t / 1e6;

    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  // pl_surf.reserve(plsize);
  for (int i = 0; i < pl_orig.size(); i++) {
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    if (i % point_filter_num == 0) {
      pl_surf.push_back(added_pt);
    }
  }
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  // UrbanNav HDL-32E clouds contain XYZIR, without per-point time. Use the
  // measured ring instead of the HDL-64E-specific vertical-angle formula.
  const auto has_field = [&](const char *name) {
    return std::any_of(msg->fields.begin(), msg->fields.end(),
        [&](const sensor_msgs::msg::PointField &field) { return field.name == name; });
  };
  if (!has_field("x") || !has_field("y") || !has_field("z") || !has_field("ring")) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("voxel_map"),
                     "Velodyne point cloud requires x, y, z and ring fields");
    return;
  }
  const bool has_time = has_field("time");
  const auto append = [&](const auto &cloud, const auto &offset_ms) {
    pl_surf.reserve(cloud.size());
    const size_t step = static_cast<size_t>(std::max(1, point_filter_num));
    for (size_t i = 0; i < cloud.size(); i += step) {
      const auto &point = cloud[i];
      if (point.ring >= N_SCANS || !std::isfinite(point.x) ||
          !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const double range_sq = double(point.x) * point.x + double(point.y) * point.y +
                              double(point.z) * point.z;
      if (range_sq <= blind * blind) continue;
      PointType added_pt{};
      added_pt.x = point.x;
      added_pt.y = point.y;
      added_pt.z = point.z;
      added_pt.intensity = point.intensity;
      added_pt.normal_x = added_pt.normal_y = added_pt.normal_z = 0;
      added_pt.curvature = offset_ms(point);
      pl_surf.push_back(added_pt);
    }
  };
  if (has_time) {
    pcl::PointCloud<velodyne_ros::Point> cloud;
    pcl::fromROSMsg(*msg, cloud);
    append(cloud, [](const velodyne_ros::Point &point) {
      return std::isfinite(point.time) && point.time >= 0 ? point.time * 1000.f : 0.f;
    });
    std::sort(pl_surf.begin(), pl_surf.end(),
              [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
  } else {
    pcl::PointCloud<velodyne_ros::PointWithoutTime> cloud;
    pcl::fromROSMsg(*msg, cloud);
    append(cloud, [](const velodyne_ros::PointWithoutTime &) { return 0.f; });
  }
}

void Preprocess::robosense_airy_handler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (msg->width == 0 || msg->height == 0) return;
  try {
    const voxelmap_input::CloudFields fields(*msg);
    const auto &x = fields.require("x"), &y = fields.require("y"), &z = fields.require("z");
    const auto &intensity = fields.require("intensity");
    const auto *ring = fields.find("ring");
    const auto *time = fields.find("timestamp");
    bool absolute = timestamp_is_absolute;
    if (!time) { time = fields.find("time"); absolute = false; }
    const double scales[] = {1.0, 1e-3, 1e-6, 1e-9};
    if (timestamp_unit < 0 || timestamp_unit > 3 || !std::isfinite(scan_rate) || scan_rate <= 0)
      throw std::invalid_argument("invalid RoboSense timestamp unit or scan rate");
    const double scale = scales[timestamp_unit];
    const double header_time = scan_start_time;
    // Like FAST_LIO_ROBOAIRY, missing ring OR time means an XYZI fallback.
    if (!ring) time = nullptr;
    const double window = 2.0 / scan_rate;
    const size_t count = size_t(msg->width)*msg->height;
    double first_time = std::numeric_limits<double>::infinity();
    double last_time = -std::numeric_limits<double>::infinity();
    bool nonzero_time = false;
    const auto valid_ring = [&](size_t i) {
      if (!ring) return true;
      const double line = fields.read(i, *ring);
      return std::isfinite(line) && line >= 0 && line < N_SCANS && std::floor(line) == line;
    };
    double anchor_time = header_time;
    if (time && absolute) {
      for (size_t i=0; i<count; ++i) {
        const double t=fields.read(i,*time)*scale;
        if (valid_ring(i) && std::isfinite(t) && t>0) { anchor_time=t; break; }
      }
    }
    const auto valid_time = [&](double t) {
      return std::isfinite(t) && t >= 0 &&
          (absolute ? std::abs(t-anchor_time) <= window : t <= window);
    };
    if (time) {
      // Use all channels before range filtering/decimation. The driver's header
      // may denote either end of a scan; absolute point times define its start.
      for (size_t i = 0; i < count; ++i) {
        if (!valid_ring(i)) continue;
        const double t = fields.read(i, *time)*scale;
        nonzero_time |= std::isfinite(t) && t != 0;
        if (!valid_time(t)) continue;
        first_time = std::min(first_time, t);
        last_time = std::max(last_time, t);
      }
      if (nonzero_time && !std::isfinite(first_time))
        throw std::invalid_argument("RoboSense point timestamps are invalid; check timestamp_unit and scan_rate");
    }
    const bool timed = time && nonzero_time && std::isfinite(first_time);
    given_offset_time = timed && last_time-first_time > 1e-9;
    if (timed && absolute) scan_start_time = first_time;
    scan_end_time = timed ? (absolute ? last_time : header_time+last_time) : scan_start_time;
    if (!given_offset_time) {
      RCLCPP_WARN_ONCE(rclcpp::get_logger("voxel_map"),
          "RoboSense has no varying point timestamps; using LiDAR-only zero offsets. Enable XYZIRT in the driver for per-point timing.");
    }
    const size_t step = size_t(std::max(1, point_filter_num));
    pl_surf.reserve(count/step+1);
    std::vector<size_t> channel_counts(static_cast<size_t>(N_SCANS),0);
    for (size_t i = 0; i < count; ++i) {
      if (!valid_ring(i)) continue;
      // rs_driver organized Airy data: width=96 channels, height=time columns.
      // Also accept the transposed layout and unorganized per-ring streams.
      size_t sample_index=i;
      if (msg->height>1 && msg->width==static_cast<uint32_t>(N_SCANS)) sample_index=i/msg->width;
      else if (msg->height==static_cast<uint32_t>(N_SCANS)) sample_index=i%msg->width;
      else if (ring) sample_index=channel_counts[static_cast<size_t>(fields.read(i,*ring))]++;
      if (sample_index%step) continue;
      const double px=fields.read(i,x), py=fields.read(i,y), pz=fields.read(i,z);
      const double reflectivity = fields.read(i,intensity);
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
          !std::isfinite(reflectivity) || px*px+py*py+pz*pz <= blind*blind ||
          px*px+py*py+pz*pz >= max_range*max_range) continue;
      double offset_ms = 0;
      if (timed) {
        const double t = fields.read(i,*time)*scale;
        if (!valid_time(t)) continue;
        offset_ms = (t - (absolute ? first_time : 0.0))*1000.0;
      } else if (time && !std::isfinite(fields.read(i,*time))) continue;
      PointType point{};
      point.x=px; point.y=py; point.z=pz; point.intensity=reflectivity;
      point.normal_x=point.normal_y=point.normal_z=0;
      point.curvature=std::max(0.0,offset_ms);
      pl_surf.push_back(point);
    }
    std::stable_sort(pl_surf.begin(),pl_surf.end(),
        [](const PointType &a,const PointType &b) { return a.curvature < b.curvature; });
  } catch (const std::exception &error) {
    pl_surf.clear();
    given_offset_time = false;
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("voxel_map"), "Invalid RoboSense cloud: %s",error.what());
  }
}
