#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "voxel_map_util.hpp"
#include "async_map_publisher.hpp"
#include <Eigen/Core>
#include <common_lib.h>
#include <csignal>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <math.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros2_utils.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <so3_math.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voxel_map/msg/states.hpp>

rclcpp::Node::SharedPtr node;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

#define INIT_TIME (0.0)
#define CALIB_ANGLE_COV (0.01)
bool calib_laser = false;
bool write_kitti_log = false;
std::string result_path = "";
// params for imu
bool imu_en = true;
std::vector<double> extrinT;
std::vector<double> extrinR;
double imu_time_offset=0.0;

// params for publish function
bool publish_voxel_map = false;
int publish_max_voxel_layer = 0;
bool publish_point_cloud = false;
int pub_point_cloud_skip = 1;

double intensity_min_thr = 0.0, intensity_max_thr = 1.0;

// params for voxel mapping algorithm
double min_eigen_value = 0.003;
int max_layer = 0;

int max_cov_points_size = 50;
int max_points_size = 50;
double sigma_num = 2.0;
double max_voxel_size = 1.0;
std::vector<int> layer_size;
// Eigen::Vector3d layer_size(20, 10, 10);
// avia
// velodyne
// Eigen::Vector3d layer_size(10, 6, 6);

// record point usage
double mean_effect_points = 0;
double mean_ds_points = 0;
double mean_raw_points = 0;

// record time
double undistort_time_mean = 0;
double down_sample_time_mean = 0;
double calc_cov_time_mean = 0;
double scan_match_time_mean = 0;
double ekf_solve_time_mean = 0;
double map_update_time_mean = 0;

mutex mtx_buffer;
condition_variable sig_buffer;
Eigen::Vector3d last_odom(0, 0, 0);
Eigen::Matrix3d last_rot = Eigen::Matrix3d::Zero();
double trajectory_len = 0;

string map_file_path, lid_topic, imu_topic;
int scanIdx = 0;

int iterCount, feats_down_size, NUM_MAX_ITERATIONS, laserCloudValidNum,
    effct_feat_num, time_log_counter, publish_count = 0;

double first_lidar_time = 0;
double lidar_end_time = 0;
double res_mean_last = 0.05;
double total_distance = 0;
double gyr_cov_scale, acc_cov_scale;
double last_timestamp_lidar, last_timestamp_imu = -1.0;
double filter_size_corner_min, filter_size_surf_min, fov_deg;
double map_incremental_time, kdtree_search_time, total_time, scan_match_time,
    solve_time;
bool lidar_pushed, flg_reset, flg_exit = false;
bool dense_map_en = true;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<double> end_time_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

// surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr cube_points_add(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr map_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudNoeffect(new PointCloudXYZI(100000, 1));
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;
V3D position_last(Zero3d);

// estimator inputs and output;
MeasureGroup Measures;
StatesGroup state;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());

void SigHandle(int sig) {
  flg_exit = true;
  RCLCPP_WARN(rclcpp::get_logger("voxel_map"), "catch sig %d", sig);
  sig_buffer.notify_all();
}

const bool intensity_contrast(PointType &x, PointType &y) {
  return (x.intensity > y.intensity);
};

const bool var_contrast(pointWithCov &x, pointWithCov &y) {
  return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};

inline void dump_lio_state_to_log(FILE *fp) {
  V3D rot_ang(Log(state.rot_end));
  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle
  fprintf(fp, "%lf %lf %lf ", state.pos_end(0), state.pos_end(1),
          state.pos_end(2)); // Pos
  fprintf(fp, "%lf %lf %lf ", state.vel_end(0), state.vel_end(1),
          state.vel_end(2)); // Vel
  fprintf(fp, "%lf %lf %lf ", state.bias_g(0), state.bias_g(1),
          state.bias_g(2)); // omega
  fprintf(fp, "%lf %lf %lf %lf ", scan_match_time, solve_time,
          map_incremental_time,
          total_time); // scan match, ekf, map incre, total
  fprintf(fp, "%zu %zu %d", feats_undistort->points.size(),
          feats_down_body->points.size(),
          effct_feat_num); // raw point number, effect number
  fprintf(fp, "\r\n");
  fflush(fp);
}

inline void kitti_log(FILE *fp) {
  Eigen::Matrix4d T_lidar_to_cam;
  T_lidar_to_cam << 0.00042768, -0.999967, -0.0080845, -0.01198, -0.00721062,
      0.0080811998, -0.99994131, -0.0540398, 0.999973864, 0.00048594,
      -0.0072069, -0.292196, 0, 0, 0, 1.0;
  V3D rot_ang(Log(state.rot_end));
  MD(4, 4) T;
  T.block<3, 3>(0, 0) = state.rot_end;
  T.block<3, 1>(0, 3) = state.pos_end;
  T(3, 0) = 0;
  T(3, 1) = 0;
  T(3, 2) = 0;
  T(3, 3) = 1;
  T = T_lidar_to_cam * T * T_lidar_to_cam.inverse();
  for (int i = 0; i < 3; i++) {
    if (i == 2)
      fprintf(fp, "%lf %lf %lf %lf", T(i, 0), T(i, 1), T(i, 2), T(i, 3));
    else
      fprintf(fp, "%lf %lf %lf %lf ", T(i, 0), T(i, 1), T(i, 2), T(i, 3));
  }
  fprintf(fp, "\n");
  // Eigen::Quaterniond q(state.rot_end);
  // fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf \r\n", state.pos_end[0],
  //         state.pos_end[1], state.pos_end[2], q.w(), q.x(), q.y(), q.z());
  fflush(fp);
}

// project the lidar scan to world frame
void pointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  p_body = p_body + Lidar_offset_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  p_body = p_body + Lidar_offset_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
  float intensity = pi->intensity;
  intensity = intensity - floor(intensity);

  int reflection_map = intensity * 10000;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(mtx_buffer);
  const double header_time = rclcpp::Time(msg->header.stamp).seconds();
  if (header_time < last_timestamp_lidar) {
    RCLCPP_ERROR(node->get_logger(), "lidar loop back, clear buffer");
    lidar_buffer.clear();
    time_buffer.clear();
    end_time_buffer.clear();
  }
  last_timestamp_lidar = header_time;
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  if (ptr->empty() || (p_pre->lidar_type == ROBOSENSE_AIRY && imu_en && !p_pre->given_offset_time)) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "Skipping empty cloud or RoboSense cloud without point timing required for IMU mode");
    return;
  }
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(p_pre->scan_start_time);
  end_time_buffer.push_back(p_pre->scan_end_time);
  sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg) {
  mtx_buffer.lock();
  // cout << "got feature" << endl;
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("voxel_map"), "lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  // RCLCPP_INFO(rclcpp::get_logger("voxel_map"), "get point cloud at time: %.6f", rclcpp::Time(msg->header.stamp).seconds());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());
  end_time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds() + (ptr->empty()?0:ptr->back().curvature/1000.0));
  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &input) {
  const auto &a=input->linear_acceleration; const auto &w=input->angular_velocity;
  if (!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||
      !std::isfinite(w.x)||!std::isfinite(w.y)||!std::isfinite(w.z)) return;
  auto msg=std::make_shared<sensor_msgs::msg::Imu>(*input);
  const double timestamp=rclcpp::Time(input->header.stamp).seconds()-imu_time_offset;
  msg->header.stamp=voxelmap_ros2::time_from_seconds(timestamp);
  std::lock_guard<std::mutex> lock(mtx_buffer);
  if (last_timestamp_imu>=0 && timestamp<last_timestamp_imu) {
    imu_buffer.clear(); flg_reset=true;
  }
  if (last_timestamp_imu>=0 && timestamp-last_timestamp_imu>.05)
    RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),5000,
        "IMU timestamp gap %.1f ms",(timestamp-last_timestamp_imu)*1000.0);
  last_timestamp_imu=timestamp;
  imu_buffer.push_back(msg);
  sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
  if (lidar_buffer.empty()) return false;
  meas.lidar=lidar_buffer.front();
  meas.lidar_beg_time=time_buffer.front();
  meas.lidar_end_time=end_time_buffer.front();
  const auto pop_scan=[&] {
    lidar_buffer.pop_front();time_buffer.pop_front();end_time_buffer.pop_front();
  };
  if (meas.lidar->empty()) { pop_scan();return false; }
  meas.imu.clear();
  if (!imu_en) { pop_scan();return true; }
  if (imu_buffer.empty() || last_timestamp_imu<meas.lidar_end_time-1e-8) return false;
  while (!imu_buffer.empty() && rclcpp::Time(imu_buffer.front()->header.stamp).seconds()<=meas.lidar_end_time) {
    meas.imu.push_back(imu_buffer.front());imu_buffer.pop_front();
  }
  // Retain the right bracket for the next scan, but also use it to interpolate
  // this scan's final partial IMU interval without extrapolating beyond a sample.
  if (!imu_buffer.empty()) meas.imu.push_back(imu_buffer.front());
  pop_scan();
  return !meas.imu.empty();
}

void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect,
                          const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPointWithCov,
                          const std::vector<ptpl> &ptpl_list) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr effect_cloud_world(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
  visualization_msgs::msg::MarkerArray ma_line;
  visualization_msgs::msg::Marker m_line;
  m_line.type = visualization_msgs::msg::Marker::LINE_LIST;
  m_line.action = visualization_msgs::msg::Marker::ADD;
  m_line.ns = "lines";
  m_line.color.a = 0.5; // Don't forget to set the alpha!
  m_line.color.r = 1.0;
  m_line.color.g = 1.0;
  m_line.color.b = 1.0;
  m_line.scale.x = 0.01;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  for (int i = 0; i < ptpl_list.size(); i++) {
    Eigen::Vector3d p_c = ptpl_list[i].point;
    Eigen::Vector3d p_w = state.rot_end * (p_c) + state.pos_end;
    pcl::PointXYZRGB pi;
    pi.x = p_w[0];
    pi.y = p_w[1];
    pi.z = p_w[2];
    // float v = laserCloudWorld->points[i].intensity / 200;
    // v = 1.0 - v;
    // uint8_t r, g, b;
    // mapJet(v, 0, 1, r, g, b);
    // pi.r = r;
    // pi.g = g;
    // pi.b = b;
    effect_cloud_world->points.push_back(pi);
    m_line.points.clear();
    geometry_msgs::msg::Point p;
    p.x = p_w[0];
    p.y = p_w[1];
    p.z = p_w[2];
    m_line.points.push_back(p);
    p.x = ptpl_list[i].center(0);
    p.y = ptpl_list[i].center(1);
    p.z = ptpl_list[i].center(2);
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  int max_num = 20000;
  for (int i = ptpl_list.size(); i < max_num; i++) {
    m_line.color.a = 0;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  pubPointWithCov->publish(ma_line);

  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*effect_cloud_world, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp =
      voxelmap_ros2::time_from_seconds(last_timestamp_lidar); //.fromSec(last_timestamp_lidar);
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_no_effect(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudNoEffect) {
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudNoeffect, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp =
      voxelmap_ros2::time_from_seconds(last_timestamp_lidar); //.fromSec(last_timestamp_lidar);
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudNoEffect->publish(laserCloudFullRes3);
}

void publish_effect(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr effect_cloud_world(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
  for (int i = 0; i < effct_feat_num; i++) {
    RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    pcl::PointXYZRGB pi;
    pi.x = laserCloudWorld->points[i].x;
    pi.y = laserCloudWorld->points[i].y;
    pi.z = laserCloudWorld->points[i].z;
    float v = laserCloudWorld->points[i].intensity / 100;
    v = 1.0 - v;
    uint8_t r, g, b;
    mapJet(v, 0, 1, r, g, b);
    pi.r = r;
    pi.g = g;
    pi.b = b;
    effect_cloud_world->points.push_back(pi);
  }

  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp =
      voxelmap_ros2::time_from_seconds(last_timestamp_lidar); //.fromSec(last_timestamp_lidar);
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void set_posestamp(T &out) {
  out.position.x = state.pos_end(0);
  out.position.y = state.pos_end(1);
  out.position.z = state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp =
      voxelmap_ros2::time_from_seconds(imu_en ? Measures.lidar_end_time : Measures.lidar_beg_time); // voxelmap_ros2::time_from_seconds(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odomAftMapped.header;
  transform.child_frame_id = odomAftMapped.child_frame_id;
  transform.transform.translation.x = state.pos_end.x();
  transform.transform.translation.y = state.pos_end.y();
  transform.transform.translation.z = state.pos_end.z();
  transform.transform.rotation = geoQuat;
  tf_broadcaster->sendTransform(transform);
  pubOdomAftMapped->publish(odomAftMapped);
}

void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher) {
  msg_body_pose.header.stamp = voxelmap_ros2::time_from_seconds(imu_en ? Measures.lidar_end_time : Measures.lidar_beg_time);
  msg_body_pose.header.frame_id = "camera_odom_frame";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = voxelmap_ros2::time_from_seconds(imu_en ? Measures.lidar_end_time : Measures.lidar_beg_time);
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  path.header.stamp = msg_body_pose.header.stamp;
  pubPath->publish(path);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("voxel_mapping_odom");
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  double ranging_cov = 0.0;
  double angle_cov = 0.0;
  std::vector<int64_t> layer_point_size;

  // cummon params
  lid_topic = node->declare_parameter<string>("common.lid_topic", "/livox/lidar");
  imu_topic = node->declare_parameter<string>("common.imu_topic", "/livox/imu");

  // noise model params
  ranging_cov = node->declare_parameter<double>("noise_model.ranging_cov", 0.02);
  angle_cov = node->declare_parameter<double>("noise_model.angle_cov", 0.05);
  gyr_cov_scale = node->declare_parameter<double>("noise_model.gyr_cov_scale", 0.1);
  acc_cov_scale = node->declare_parameter<double>("noise_model.acc_cov_scale", 0.1);

  // IMU parameters; the Airy preset uses the calibrated LiDAR->IMU transform.
  imu_en = node->declare_parameter<bool>("imu.imu_en", false);
  imu_time_offset = node->declare_parameter<double>("common.time_offset_lidar_to_imu",0.0);
  const auto gravity_ref = node->declare_parameter<std::vector<double>>("imu.gravity_ref",std::vector<double>{0.0,0.0,0.0});
  const double acc_bias_cov=node->declare_parameter<double>("noise_model.b_acc_cov",.00001);
  const double gyr_bias_cov=node->declare_parameter<double>("noise_model.b_gyr_cov",.00001);

  extrinT = node->declare_parameter<vector<double>>("imu.extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
  extrinR = node->declare_parameter<vector<double>>("imu.extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

  // mapping algorithm params
  NUM_MAX_ITERATIONS = node->declare_parameter<int>("mapping.max_iteration", 4);
  max_points_size = node->declare_parameter<int>("mapping.max_points_size", 100);
  max_cov_points_size = node->declare_parameter<int>("mapping.max_cov_points_size", 100);
  layer_point_size = node->declare_parameter<std::vector<int64_t>>("mapping.layer_point_size", std::vector<int64_t>{5, 5, 5, 5, 5});
  max_layer = node->declare_parameter<int>("mapping.max_layer", 2);
  max_voxel_size = node->declare_parameter<double>("mapping.voxel_size", 1.0);
  filter_size_surf_min = node->declare_parameter<double>("mapping.down_sample_size", 0.5);
  // std::cout << "filter_size_surf_min:" << filter_size_surf_min << std::endl;
  min_eigen_value = node->declare_parameter<double>("mapping.plannar_threshold", 0.01);

  // preprocess params
  p_pre->blind = node->declare_parameter<double>("preprocess.blind", 0.01);
  calib_laser = node->declare_parameter<bool>("preprocess.calib_laser", false);
  p_pre->lidar_type = node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  p_pre->N_SCANS = node->declare_parameter<int>("preprocess.scan_line", 16);
  p_pre->point_filter_num = node->declare_parameter<int>("preprocess.point_filter_num", 2);
  p_pre->timestamp_unit = node->declare_parameter<int>("preprocess.timestamp_unit", 0);
  p_pre->timestamp_is_absolute = node->declare_parameter<bool>("preprocess.timestamp_is_absolute", true);
  p_pre->scan_rate = node->declare_parameter<double>("preprocess.scan_rate", 10.0);
  p_pre->max_range = node->declare_parameter<double>("preprocess.max_range",150.0);


  // visualization params
  publish_voxel_map = node->declare_parameter<bool>("visualization.pub_voxel_map", false);
  publish_max_voxel_layer = node->declare_parameter<int>("visualization.publish_max_voxel_layer", 0);
  publish_point_cloud = node->declare_parameter<bool>("visualization.pub_point_cloud", true);
  pub_point_cloud_skip = node->declare_parameter<int>("visualization.pub_point_cloud_skip", 1);
  dense_map_en = node->declare_parameter<bool>("visualization.dense_map_enable", false);
  const bool publish_global_map = node->declare_parameter<bool>("visualization.pub_global_map", false);
  const double display_resolution = node->declare_parameter<double>("visualization.global_map_voxel_size", 0.5);
  const double display_period = node->declare_parameter<double>("visualization.map_publish_period", 2.0);
  const auto max_display_points = node->declare_parameter<int64_t>("visualization.max_global_map_points", 100000);
  const auto max_display_planes = node->declare_parameter<int64_t>("visualization.max_voxel_planes", 5000);
  if (!std::isfinite(display_resolution) || display_resolution <= 0 ||
      !std::isfinite(display_period) || display_period < 0.05 ||
      max_display_points < 8 || max_display_planes < 1 || pub_point_cloud_skip < 1 ||
      publish_max_voxel_layer < 0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid visualization resolution, period, limits or point skip");
    rclcpp::shutdown();
    return 1;
  }


  // result params
  write_kitti_log = node->declare_parameter<bool>("Result.write_kitti_log", false);
  result_path = node->declare_parameter<string>("Result.result_path", "");
  cout << "p_pre->lidar_type " << p_pre->lidar_type << endl;
  for (int i = 0; i < layer_point_size.size(); i++) {
    layer_size.push_back(layer_point_size[i]);
  }

  if (gravity_ref.size()!=3 || !std::isfinite(imu_time_offset) || acc_bias_cov<0 || gyr_bias_cov<0 ||
      !std::isfinite(p_pre->max_range) || p_pre->max_range<=p_pre->blind || extrinT.size() != 3 || extrinR.size() != 9 || max_layer < 0 ||
      layer_size.size() <= static_cast<size_t>(max_layer) ||
      filter_size_surf_min <= 0 || max_voxel_size <= 0 ||
      NUM_MAX_ITERATIONS < 1 || p_pre->point_filter_num < 1 ||
      p_pre->N_SCANS < 1 || p_pre->N_SCANS > 128 ||
      p_pre->timestamp_unit < 0 || p_pre->timestamp_unit > 3 ||
      !std::isfinite(p_pre->scan_rate) || p_pre->scan_rate <= 0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid extrinsics, voxel layers, iterations, or preprocessing sizes");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);
  if (p_pre->lidar_type == AVIA) {
    sub_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, sensor_qos, livox_pcl_cbk);
  } else {
    sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, sensor_qos, standard_pcl_cbk);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  if (imu_en) {
  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(4000), imu_cbk);
  }
  const auto display_qos = rclcpp::QoS(1).best_effort();
  const auto snapshot_qos = rclcpp::QoS(1).best_effort().transient_local();
  auto global_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", snapshot_qos);
  auto pubLaserCloudFullRes = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", display_qos);
  auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", display_qos);
  auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", rclcpp::QoS(10));
  auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", display_qos);
  auto voxel_map_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", snapshot_qos);

  path.header.stamp = voxelmap_ros2::time_from_seconds(last_timestamp_lidar);
  path.header.frame_id = "camera_init";

  /*** variables definition ***/
  VD(DIM_STATE) solution;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  V3D rot_add, t_add;
  StatesGroup state_propagat;
  PointType pointOri, pointSel, coeff;
  PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
  int frame_num = 0;
  double deltaT, deltaR, aver_time_consu = 0;
  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0,
                                          is_first_frame = true;
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);

  shared_ptr<ImuProcess> p_imu(new ImuProcess());
  p_imu->imu_en = imu_en;
  Eigen::Vector3d extT;
  Eigen::Matrix3d extR;
  extT << extrinT[0], extrinT[1], extrinT[2];
  extR << extrinR[0], extrinR[1], extrinR[2], extrinR[3], extrinR[4],
      extrinR[5], extrinR[6], extrinR[7], extrinR[8];
  p_imu->set_extrinsic(extT, extR);
  if (!extR.allFinite() || !extT.allFinite() ||
      !(extR.transpose()*extR).isApprox(M3D::Identity(),1e-6) || std::abs(extR.determinant()-1)>1e-6) {
    RCLCPP_ERROR(node->get_logger(),"imu.extrinsic_R must be a proper rotation");
    rclcpp::shutdown();return 1;
  }
  p_imu->set_gravity_ref(V3D(gravity_ref[0],gravity_ref[1],gravity_ref[2]));
  // Point covariance is computed in the LiDAR frame, then rotated into the
  // scan-end body frame consumed by the measurement model in IMU mode.
  const auto measurement_cov = [&](const V3D &point,double angle) {
    V3D sensor_point=imu_en ? V3D(extR.transpose()*(point-extT)) : point;
    M3D covariance;calcBodyCov(sensor_point,ranging_cov,angle,covariance);
    if (imu_en) covariance=(extR*covariance*extR.transpose()).eval();
    return covariance;
  };


  // IMU mode outputs deskewed points in the scan-end IMU body frame.
  if (imu_en) {
    std::cout << "use imu" << std::endl;
  } else {
    std::cout << "no imu" << std::endl;
  }

  p_imu->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
  p_imu->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
  p_imu->set_gyr_bias_cov(V3D(gyr_bias_cov,gyr_bias_cov,gyr_bias_cov));
  p_imu->set_acc_bias_cov(V3D(acc_bias_cov,acc_bias_cov,acc_bias_cov));

  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  /*** debug record ***/
  FILE *fp_kitti = nullptr;

  if (write_kitti_log) {
    fp_kitti = fopen(result_path.c_str(), "w");
    if (!fp_kitti) {
      RCLCPP_ERROR(node->get_logger(), "Cannot open trajectory file: %s", result_path.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  rclcpp::WallRate rate(5000);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // for Plane Map
  bool init_map = false;
  std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
  last_rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  voxelmap_display::AsyncMapPublisher map_display(publish_global_map, publish_voxel_map,
      display_resolution, display_period, static_cast<size_t>(max_display_points),
      static_cast<size_t>(max_display_planes), global_map_pub, voxel_map_pub);
  voxelmap_display::AsyncScanPublisher scan_display(pubLaserCloudFullRes);
  const auto queue_map = [&](const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &world) {
    if (!map_display.due()) return;
    voxelmap_display::AsyncMapPublisher::Snapshot snapshot;
    snapshot.world_cloud = world;
    snapshot.stamp = voxelmap_ros2::time_from_seconds(imu_en ? Measures.lidar_end_time : Measures.lidar_beg_time);
    if (publish_voxel_map) {
      for (const auto &entry : voxel_map)
        voxelmap_display::collect_planes(entry.second, publish_max_voxel_layer, snapshot.planes);
    }
    map_display.submit(std::move(snapshot));
  };


  while (rclcpp::ok()) {
    if (flg_exit)
      break;
    executor.spin_some();
    if (!rclcpp::ok()) break;
    if (sync_packages(Measures)) {
      // std::cout << "sync once" << std::endl;
      if (flg_reset) {
        RCLCPP_WARN(rclcpp::get_logger("voxel_map"), "reset when rosbag play back");
        p_imu->Reset();
        flg_reset = false;
        continue;
      }
      std::cout << "scanIdx:" << scanIdx << std::endl;
      double t0, t1, t2, t3, t4, t5, match_start, match_time, solve_start,
          svd_time;
      match_time = 0;
      solve_time = 0;
      svd_time = 0;

      // std::cout << " init rot cov:" << std::endl
      //           << state.cov.block<3, 3>(0, 0) << std::endl;
      auto undistort_start = std::chrono::high_resolution_clock::now();
      p_imu->Process(Measures, state, feats_undistort);
      auto undistort_end = std::chrono::high_resolution_clock::now();
      auto undistort_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              undistort_end - undistort_start)
              .count() *
          1000;
      if (calib_laser) {
        // calib the vertical angle for kitti dataset
        for (size_t i = 0; i < feats_undistort->size(); i++) {
          PointType pi = feats_undistort->points[i];
          double range = sqrt(pi.x * pi.x + pi.y * pi.y + pi.z * pi.z);
          double calib_vertical_angle = deg2rad(0.15);
          double vertical_angle = asin(pi.z / range) + calib_vertical_angle;
          double horizon_angle = atan2(pi.y, pi.x);
          pi.z = range * sin(vertical_angle);
          double project_len = range * cos(vertical_angle);
          pi.x = project_len * cos(horizon_angle);
          pi.y = project_len * sin(horizon_angle);
          feats_undistort->points[i] = pi;
        }
      }
      state_propagat = state;

      if (is_first_frame) {
        first_lidar_time = Measures.lidar_beg_time;
        is_first_frame = false;
      }

      if (feats_undistort->empty() || (feats_undistort == NULL)) {
        p_imu->first_lidar_time = first_lidar_time;
        cout << "FAST-LIO not ready" << endl;
        continue;
      }

      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME
                           ? false
                           : true;
      if (flg_EKF_inited && !init_map) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
            new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Quaterniond q(state.rot_end);
        transformLidar(state, p_imu, feats_undistort, world_lidar);
        std::vector<pointWithCov> pv_list;
        for (size_t i = 0; i < world_lidar->size(); i++) {
          pointWithCov pv;
          pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
              world_lidar->points[i].z;
          V3D point_this(feats_undistort->points[i].x,
                         feats_undistort->points[i].y,
                         feats_undistort->points[i].z);
          M3D cov=measurement_cov(point_this,angle_cov);
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          cov = state.rot_end * cov * state.rot_end.transpose() +
                (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                    (-point_crossmat).transpose() +
                state.cov.block<3, 3>(3, 3);
          pv.cov = cov;
          pv_list.push_back(pv);
          Eigen::Vector3d sigma_pv = pv.cov.diagonal();
          sigma_pv[0] = sqrt(sigma_pv[0]);
          sigma_pv[1] = sqrt(sigma_pv[1]);
          sigma_pv[2] = sqrt(sigma_pv[2]);
        }

        buildVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                      max_points_size, max_points_size, min_eigen_value,
                      voxel_map);
        std::cout << "build voxel map" << std::endl;
        if (write_kitti_log) {
          kitti_log(fp_kitti);
        }

        scanIdx++;
        queue_map(world_lidar);
        init_map = true;
        continue;
      }

      /*** downsample the feature points in a scan ***/
      auto t_downsample_start = std::chrono::high_resolution_clock::now();
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      auto t_downsample_end = std::chrono::high_resolution_clock::now();
      std::cout << " feats size:" << feats_undistort->size()
                << ", down size:" << feats_down_body->size() << std::endl;
      auto t_downsample =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              t_downsample_end - t_downsample_start)
              .count() *
          1000;

      sort(feats_down_body->points.begin(), feats_down_body->points.end(),
           time_list);

      int rematch_num = 0;
      bool nearest_search_en = true;
      double total_residual;

      scan_match_time = 0.0;

      std::vector<M3D> body_var;
      std::vector<M3D> crossmat_list;

      /*** iterated state estimation ***/
      auto calc_point_cov_start = std::chrono::high_resolution_clock::now();
      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(feats_down_body->points[i].x,
                       feats_down_body->points[i].y,
                       feats_down_body->points[i].z);
        M3D cov=measurement_cov(point_this,angle_cov);
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        crossmat_list.push_back(point_crossmat);
        M3D rot_var = state.cov.block<3, 3>(0, 0);
        M3D t_var = state.cov.block<3, 3>(3, 3);
        body_var.push_back(cov);
      }
      auto calc_point_cov_end = std::chrono::high_resolution_clock::now();
      double calc_point_cov_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              calc_point_cov_end - calc_point_cov_start)
              .count() *
          1000;

      for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
        laserCloudOri->clear();
        laserCloudNoeffect->clear();
        corr_normvect->clear();
        total_residual = 0.0;

        std::vector<double> r_list;
        std::vector<ptpl> ptpl_list;
        /** LiDAR match based on 3 sigma criterion **/

        vector<pointWithCov> pv_list;
        std::vector<M3D> var_list;
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
            new pcl::PointCloud<pcl::PointXYZI>);
        transformLidar(state, p_imu, feats_down_body, world_lidar);
        for (size_t i = 0; i < feats_down_body->size(); i++) {
          pointWithCov pv;
          pv.point << feats_down_body->points[i].x,
              feats_down_body->points[i].y, feats_down_body->points[i].z;
          pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y,
              world_lidar->points[i].z;
          M3D cov = body_var[i];
          M3D point_crossmat = crossmat_list[i];
          M3D rot_var = state.cov.block<3, 3>(0, 0);
          M3D t_var = state.cov.block<3, 3>(3, 3);
          cov = state.rot_end * cov * state.rot_end.transpose() +
                (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
                t_var;
          pv.cov = cov;
          pv_list.push_back(pv);
          var_list.push_back(cov);
        }
        auto scan_match_time_start = std::chrono::high_resolution_clock::now();
        std::vector<V3D> non_match_list;
        BuildResidualListOMP(voxel_map, max_voxel_size, 3.0, max_layer, pv_list,
                             ptpl_list, non_match_list);

        auto scan_match_time_end = std::chrono::high_resolution_clock::now();

        effct_feat_num = 0;
        for (int i = 0; i < ptpl_list.size(); i++) {
          PointType pi_body;
          PointType pi_world;
          PointType pl;
          pi_body.x = ptpl_list[i].point(0);
          pi_body.y = ptpl_list[i].point(1);
          pi_body.z = ptpl_list[i].point(2);
          pointBodyToWorld(&pi_body, &pi_world);
          pl.x = ptpl_list[i].normal(0);
          pl.y = ptpl_list[i].normal(1);
          pl.z = ptpl_list[i].normal(2);
          effct_feat_num++;
          float dis = (pi_world.x * pl.x + pi_world.y * pl.y +
                       pi_world.z * pl.z + ptpl_list[i].d);
          pl.intensity = dis;
          laserCloudOri->push_back(pi_body);
          corr_normvect->push_back(pl);
          total_residual += fabs(dis);
        }
        res_mean_last = total_residual / effct_feat_num;
        scan_match_time +=
            std::chrono::duration_cast<std::chrono::duration<double>>(
                scan_match_time_end - scan_match_time_start)
                .count() *
            1000;

        // cout << "[ Matching ]: Time:"
        //      << std::chrono::duration_cast<std::chrono::duration<double>>(
        //             scan_match_time_end - scan_match_time_start)
        //                 .count() *
        //             1000
        //      << " ms  Effective feature num: " << effct_feat_num
        //      << " All num:" << feats_down_body->size() << "  res_mean_last "
        //      << res_mean_last << endl;

        auto t_solve_start = std::chrono::high_resolution_clock::now();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector
         * ***/
        MatrixXd Hsub(effct_feat_num, 6);
        MatrixXd Hsub_T_R_inv(6, effct_feat_num);
        VectorXd R_inv(effct_feat_num);
        VectorXd meas_vec(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++) {
          const PointType &laser_p = laserCloudOri->points[i];
          V3D point_this(laser_p.x, laser_p.y, laser_p.z);
          M3D cov=measurement_cov(point_this,calib_laser ? CALIB_ANGLE_COV : angle_cov);

          cov = state.rot_end * cov * state.rot_end.transpose();
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          const PointType &norm_p = corr_normvect->points[i];
          V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
          V3D point_world = state.rot_end * point_this + state.pos_end;
          // /*** get the normal vector of closest surface/corner ***/
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
          J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
          double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
          R_inv(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
          double ranging_dis = point_this.norm();
          laserCloudOri->points[i].intensity = sqrt(R_inv(i));
          laserCloudOri->points[i].normal_x =
              corr_normvect->points[i].intensity;
          laserCloudOri->points[i].normal_y = sqrt(sigma_l);
          laserCloudOri->points[i].normal_z =
              sqrt(norm_vec.transpose() * cov * norm_vec);
          laserCloudOri->points[i].curvature =
              sqrt(sigma_l + norm_vec.transpose() * cov * norm_vec);

          /*** calculate the Measuremnt Jacobian matrix H ***/
          V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
          Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
          Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
              A[2] * R_inv(i), norm_p.x * R_inv(i), norm_p.y * R_inv(i),
              norm_p.z * R_inv(i);
          /*** Measuremnt: distance to the closest surface/corner ***/
          meas_vec(i) = -norm_p.intensity;
        }
        MatrixXd K(DIM_STATE, effct_feat_num);

        EKF_stop_flg = false;
        flg_EKF_converged = false;

        /*** Iterative Kalman Filter Update ***/
        if (!flg_EKF_inited) {
          cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
          /*** only run in initialization period ***/
          MatrixXd H_init(MD(9, DIM_STATE)::Zero());
          MatrixXd z_init(VD(9)::Zero());
          H_init.block<3, 3>(0, 0) = M3D::Identity();
          H_init.block<3, 3>(3, 3) = M3D::Identity();
          H_init.block<3, 3>(6, 15) = M3D::Identity();
          z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
          z_init.block<3, 1>(0, 0) = -state.pos_end;

          auto H_init_T = H_init.transpose();
          auto &&K_init =
              state.cov * H_init_T *
              (H_init * state.cov * H_init_T + 0.0001 * MD(9, 9)::Identity())
                  .inverse();
          solution = K_init * z_init;

          state.resetpose();
          EKF_stop_flg = true;
        } else {
          auto &&Hsub_T = Hsub.transpose();
          H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
          MD(DIM_STATE, DIM_STATE) &&K_1 =
              (H_T_H + (state.cov).inverse()).inverse();
          K = K_1.block<DIM_STATE, 6>(0, 0) * Hsub_T_R_inv;
          auto vec = state_propagat - state;
          solution = K * meas_vec + vec - K * Hsub * vec.block<6, 1>(0, 0);

          int minRow, minCol;
          if (0) // if(V.minCoeff(&minRow, &minCol) < 1.0f)
          {
            VD(6) V = H_T_H.block<6, 6>(0, 0).eigenvalues().real();
            cout << "!!!!!! Degeneration Happend, eigen values: "
                 << V.transpose() << endl;
            EKF_stop_flg = true;
            solution.block<6, 1>(9, 0).setZero();
          }

          state += solution;

          rot_add = solution.block<3, 1>(0, 0);
          t_add = solution.block<3, 1>(3, 0);

          if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) {
            flg_EKF_converged = true;
          }

          deltaR = rot_add.norm() * 57.3;
          deltaT = t_add.norm() * 100;
        }
        euler_cur = RotMtoEuler(state.rot_end);
        /*** Rematch Judgement ***/
        nearest_search_en = false;
        if (flg_EKF_converged ||
            ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)))) {
          nearest_search_en = true;
          rematch_num++;
        }

        /*** Convergence Judgements and Covariance Update ***/
        if (!EKF_stop_flg &&
            (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))) {
          if (flg_EKF_inited) {
            /*** Covariance Update ***/
            G.setZero();
            G.block<DIM_STATE, 6>(0, 0) = K * Hsub;
            state.cov = (I_STATE - G) * state.cov;
            total_distance += (state.pos_end - position_last).norm();
            position_last = state.pos_end;

            geoQuat = voxelmap_ros2::quaternion_from_rpy(
                euler_cur(0), euler_cur(1), euler_cur(2));

            VD(DIM_STATE) K_sum = K.rowwise().sum();
            VD(DIM_STATE) P_diag = state.cov.diagonal();
          }
          EKF_stop_flg = true;
        }
        auto t_solve_end = std::chrono::high_resolution_clock::now();
        solve_time += std::chrono::duration_cast<std::chrono::duration<double>>(
                          t_solve_end - t_solve_start)
                          .count() *
                      1000;

        if (EKF_stop_flg)
          break;
      }

      /*** add the  points to the voxel map ***/
      auto map_incremental_start = std::chrono::high_resolution_clock::now();
      pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
          new pcl::PointCloud<pcl::PointXYZI>);
      transformLidar(state, p_imu, feats_down_body, world_lidar);
      std::vector<pointWithCov> pv_list;
      for (size_t i = 0; i < world_lidar->size(); i++) {
        pointWithCov pv;
        pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        M3D point_crossmat = crossmat_list[i];
        M3D cov = body_var[i];
        cov = state.rot_end * cov * state.rot_end.transpose() +
              (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                  (-point_crossmat).transpose() +
              state.cov.block<3, 3>(3, 3);
        pv.cov = cov;
        pv_list.push_back(pv);
      }
      std::sort(pv_list.begin(), pv_list.end(), var_contrast);
      updateVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                     max_points_size, max_points_size, min_eigen_value,
                     voxel_map);
      auto map_incremental_end = std::chrono::high_resolution_clock::now();
      map_incremental_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              map_incremental_end - map_incremental_start)
              .count() *
          1000;

      total_time = t_downsample + scan_match_time + solve_time +
                   map_incremental_time + undistort_time + calc_point_cov_time;
      /******* Publish functions:  *******/
      publish_odometry(pubOdomAftMapped);
      publish_path(pubPath);
      queue_map(world_lidar);
      if (publish_point_cloud && scan_display.ready()) {
        voxelmap_display::AsyncScanPublisher::Snapshot snapshot;
        const auto source = dense_map_en ? feats_undistort : feats_down_body;
        snapshot.cloud.reset(new PointCloudXYZI(*source));
        snapshot.rotation = state.rot_end;
        snapshot.position = state.pos_end;
        snapshot.point_skip = static_cast<size_t>(pub_point_cloud_skip);
        snapshot.stamp = voxelmap_ros2::time_from_seconds(imu_en ? Measures.lidar_end_time : Measures.lidar_beg_time);
        scan_display.submit(std::move(snapshot));
      }
      if (pubLaserCloudEffect->get_subscription_count() > 0) publish_effect(pubLaserCloudEffect);

      frame_num++;
      mean_raw_points = mean_raw_points * (frame_num - 1) / frame_num +
                        (double)(feats_undistort->size()) / frame_num;
      mean_ds_points = mean_ds_points * (frame_num - 1) / frame_num +
                       (double)(feats_down_body->size()) / frame_num;
      mean_effect_points = mean_effect_points * (frame_num - 1) / frame_num +
                           (double)effct_feat_num / frame_num;

      undistort_time_mean = undistort_time_mean * (frame_num - 1) / frame_num +
                            (undistort_time) / frame_num;
      down_sample_time_mean =
          down_sample_time_mean * (frame_num - 1) / frame_num +
          (t_downsample) / frame_num;
      calc_cov_time_mean = calc_cov_time_mean * (frame_num - 1) / frame_num +
                           (calc_point_cov_time) / frame_num;
      scan_match_time_mean =
          scan_match_time_mean * (frame_num - 1) / frame_num +
          (scan_match_time) / frame_num;
      ekf_solve_time_mean = ekf_solve_time_mean * (frame_num - 1) / frame_num +
                            (solve_time) / frame_num;
      map_update_time_mean =
          map_update_time_mean * (frame_num - 1) / frame_num +
          (map_incremental_time) / frame_num;

      aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num +
                        (total_time) / frame_num;

      time_log_counter++;
      cout << "[ Time ]: "
           << "average undistort: " << undistort_time_mean << std::endl;
      cout << "[ Time ]: "
           << "average down sample: " << down_sample_time_mean << std::endl;
      cout << "[ Time ]: "
           << "average calc cov: " << calc_cov_time_mean << std::endl;
      cout << "[ Time ]: "
           << "average scan match: " << scan_match_time_mean << std::endl;
      cout << "[ Time ]: "
           << "average solve: " << ekf_solve_time_mean << std::endl;
      cout << "[ Time ]: "
           << "average map incremental: " << map_update_time_mean << std::endl;
      cout << "[ Time ]: "
           << " average total " << aver_time_consu << endl;

      if (write_kitti_log) {
        kitti_log(fp_kitti);
      }

      scanIdx++;
    }

    rate.sleep();
  }
  scan_display.stop();
  map_display.stop();
  if (fp_kitti) std::fclose(fp_kitti);
  rclcpp::shutdown();
  return 0;
}
