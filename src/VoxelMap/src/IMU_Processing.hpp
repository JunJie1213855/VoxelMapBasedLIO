#include <Eigen/Eigen>
#include <cmath>
#include <common_lib.h>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros2_utils.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <so3_math.h>
#include <thread>
#include <voxel_map/msg/states.hpp>

/// *************Preconfiguration

#define MAX_INI_COUNT (200)

inline bool time_list(const PointType &x, const PointType &y) {
  return (x.curvature < y.curvature);
};

/// *************IMU Process and undistortion
class ImuProcess {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gravity_ref(const V3D &reference) { gravity_ref_ = reference; }
  bool initialized() const { return !imu_need_init_; }
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void Process(const MeasureGroup &meas, StatesGroup &state,
               PointCloudXYZI::Ptr &pcl_un_);


  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  double first_lidar_time;
  bool imu_en;

private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  void UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout,
                    PointCloudXYZI &pcl_in_out);
  void only_propag(const MeasureGroup &meas, StatesGroup &state_inout,
                   PointCloudXYZI::Ptr &pcl_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D> v_rot_pcl_;

  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  V3D gravity_ref_ = Zero3d;
  double last_lidar_end_time_ = -1.0;
  double start_timestamp_;
  double time_last_scan_;
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
  imu_en = true;
  init_iter_num = 1;
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  // old
  cov_acc_scale = V3D(1, 1, 1);
  cov_gyr_scale = V3D(1, 1, 1);

  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
  RCLCPP_WARN(rclcpp::get_logger("voxel_map"), "Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  last_lidar_end_time_ = -1;
  b_first_frame_ = true;
  init_iter_num = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T) {
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale(const V3D &scaler) {
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov_scale(const V3D &scaler) {
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout,
                          int &N) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  RCLCPP_INFO(rclcpp::get_logger("voxel_map"), "IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    N = 0;
    b_first_frame_ = false;
    mean_acc.setZero(); mean_gyr.setZero();
    first_lidar_time = meas.lidar_beg_time;
  }
  for (const auto &imu : meas.imu) {
    if (rclcpp::Time(imu->header.stamp).seconds() > meas.lidar_end_time) break;
    const auto &a=imu->linear_acceleration; const auto &w=imu->angular_velocity;
    cur_acc << a.x,a.y,a.z; cur_gyr << w.x,w.y,w.z;
    ++N;
    mean_acc += (cur_acc-mean_acc)/N;
    mean_gyr += (cur_gyr-mean_gyr)/N;
    last_imu_ = imu;
  }
  if (mean_acc.norm() < 1e-6 || !mean_acc.allFinite()) return;
  if (gravity_ref_.norm() > 1e-6) {
    state_inout.gravity = gravity_ref_.normalized()*G_m_s2;
    state_inout.rot_end = Eigen::Quaterniond::FromTwoVectors(
        V3D(-mean_acc.normalized()), V3D(gravity_ref_.normalized())).toRotationMatrix();
  } else {
    state_inout.gravity = -mean_acc.normalized()*G_m_s2;
    state_inout.rot_end.setIdentity();
  }
  state_inout.bias_g = mean_gyr;
  last_lidar_end_time_ = meas.lidar_end_time;

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas,
                              StatesGroup &state_inout,
                              PointCloudXYZI &pcl_out) {
  if (meas.lidar->empty() || meas.imu.empty()) return;
  const double end_time=meas.lidar_end_time;
  const double start_time=last_lidar_end_time_;
  auto samples=meas.imu;
  if (last_imu_ && rclcpp::Time(last_imu_->header.stamp).seconds() <
                   rclcpp::Time(samples.front()->header.stamp).seconds()) samples.push_front(last_imu_);
  const auto stamp=[](const auto &imu) { return rclcpp::Time(imu->header.stamp).seconds(); };
  if (samples.size()<2 || start_time<0 || end_time<=start_time ||
      stamp(samples.front())>start_time+1e-8 || stamp(samples.back())<end_time-1e-8) {
    RCLCPP_WARN(rclcpp::get_logger("voxel_map"),"Insufficient IMU coverage for scan; skip deskew");
    return;
  }
  struct Segment {
    double time;
    M3D rotation;
    V3D position,velocity,acceleration,gyro;
  };
  std::vector<Segment,Eigen::aligned_allocator<Segment>> trajectory;
  M3D rotation=state_inout.rot_end;
  V3D position=state_inout.pos_end, velocity=state_inout.vel_end;
  for (size_t i=0;i+1<samples.size();++i) {
    const double a=stamp(samples[i]),b=stamp(samples[i+1]);
    const double lo=std::max(start_time,a),hi=std::min(end_time,b);
    if (b<=a || hi<=lo) continue;
    const double ratio=((lo+hi)*.5-a)/(b-a);
    const auto &h=*samples[i]; const auto &t=*samples[i+1];
    V3D gyro=V3D(h.angular_velocity.x,h.angular_velocity.y,h.angular_velocity.z)*(1-ratio)+
             V3D(t.angular_velocity.x,t.angular_velocity.y,t.angular_velocity.z)*ratio-state_inout.bias_g;
    V3D acceleration=(V3D(h.linear_acceleration.x,h.linear_acceleration.y,h.linear_acceleration.z)*(1-ratio)+
             V3D(t.linear_acceleration.x,t.linear_acceleration.y,t.linear_acceleration.z)*ratio)*
             (G_m_s2/mean_acc.norm())-state_inout.bias_a;
    const double dt=hi-lo;
    const M3D mid_rotation=rotation*Exp(gyro,dt*.5);
    const V3D world_acc=mid_rotation*acceleration+state_inout.gravity;
    trajectory.push_back({lo,rotation,position,velocity,world_acc,gyro});
    MD(DIM_STATE,DIM_STATE) transition=MD(DIM_STATE,DIM_STATE)::Identity();
    MD(DIM_STATE,DIM_STATE) noise=MD(DIM_STATE,DIM_STATE)::Zero();
    M3D skew; skew << SKEW_SYM_MATRX(acceleration);
    transition.block<3,3>(0,0)=Exp(gyro,-dt);
    transition.block<3,3>(0,9)=-Eye3d*dt;
    transition.block<3,3>(3,6)=Eye3d*dt;
    transition.block<3,3>(6,0)=-mid_rotation*skew*dt;
    transition.block<3,3>(6,12)=-mid_rotation*dt;
    transition.block<3,3>(6,15)=Eye3d*dt;
    noise.block<3,3>(0,0).diagonal()=cov_gyr*dt*dt;
    noise.block<3,3>(6,6)=mid_rotation*cov_acc.asDiagonal()*mid_rotation.transpose()*dt*dt;
    noise.block<3,3>(9,9).diagonal()=cov_bias_gyr*dt*dt;
    noise.block<3,3>(12,12).diagonal()=cov_bias_acc*dt*dt;
    state_inout.cov=transition*state_inout.cov*transition.transpose()+noise;
    position+=velocity*dt+.5*world_acc*dt*dt;
    velocity+=world_acc*dt;
    rotation=rotation*Exp(gyro,dt);
  }
  if (trajectory.empty()) return;
  state_inout.rot_end=rotation;
  state_inout.pos_end=position;
  state_inout.vel_end=velocity;
  last_lidar_end_time_=end_time;
  for (const auto &imu:samples) if (stamp(imu)<=end_time) last_imu_=imu;
  // VoxelMap's measurement Jacobian consumes body-frame points. Apply the full
  // LiDAR->IMU extrinsic here exactly once, then deskew to the scan-end body pose.
  pcl_out=*meas.lidar;
  std::stable_sort(pcl_out.begin(),pcl_out.end(),time_list);
  size_t segment=0;
  for (auto &point:pcl_out) {
    const double time=meas.lidar_beg_time+point.curvature/1000.0;
    while (segment+1<trajectory.size() && trajectory[segment+1].time<=time) ++segment;
    const auto &pose=trajectory[segment];
    const double dt=time-pose.time;
    const M3D at_rotation=pose.rotation*Exp(pose.gyro,dt);
    const V3D at_position=pose.position+pose.velocity*dt+.5*pose.acceleration*dt*dt;
    const V3D body=Lid_rot_to_IMU*V3D(point.x,point.y,point.z)+Lid_offset_to_IMU;
    const V3D corrected=rotation.transpose()*(at_rotation*body+at_position-position);
    point.x=corrected.x();point.y=corrected.y();point.z=corrected.z();
  }
}

// constant velocity model
void ImuProcess::only_propag(const MeasureGroup &meas, StatesGroup &state_inout,
                             PointCloudXYZI::Ptr &pcl_out) {
  const double &pcl_beg_time = meas.lidar_beg_time;

  /*** sort point clouds by offset time ***/
  pcl_out = meas.lidar;
  const double &pcl_end_time =
      pcl_beg_time + pcl_out->points.back().curvature / double(1000);

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  if (b_first_frame_) {
    dt = 0.1;
    b_first_frame_ = false;
    time_last_scan_ = pcl_beg_time;
  } else {
    dt = pcl_beg_time - time_last_scan_;
    time_last_scan_ = pcl_beg_time;
  }

  /* covariance propagation */
  // M3D acc_avr_skew;
  M3D Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 9) = Eye3d * dt;
  F_x.block<3, 3>(3, 6) = Eye3d * dt;
  cov_w.block<3, 3>(9, 9).diagonal() =
      cov_gyr * dt * dt; // for omega in constant model
  cov_w.block<3, 3>(6, 6).diagonal() =
      cov_acc * dt * dt; // for velocity in constant model

  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;
}

void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat,
                         PointCloudXYZI::Ptr &cur_pcl_un_) {
  double t1, t2, t3;

  if (imu_en) cur_pcl_un_->clear();
  if (meas.imu.empty() && imu_en) {
    return;
  }
  assert(meas.lidar != nullptr);

  if (imu_need_init_ && imu_en) {
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init_ = true;

    if (init_iter_num > MAX_INI_COUNT && mean_acc.norm()>1e-6) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;
      stat.cov.setIdentity();
      stat.cov.block<3,3>(6,6) *= 1e-3;
      stat.cov.block<3,3>(9,9) *= 1e-5;
      stat.cov.block<3,3>(12,12) *= 1e-4;
      stat.cov.block<3,3>(15,15) *= 1e-3;
      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      RCLCPP_INFO(rclcpp::get_logger("voxel_map"),
          "IMU initialization complete (%d samples): gravity [%.4f %.4f %.4f], gyro bias [%.5f %.5f %.5f]",
          init_iter_num,stat.gravity.x(),stat.gravity.y(),stat.gravity.z(),
          stat.bias_g.x(),stat.bias_g.y(),stat.bias_g.z());
    }

    return;
  }

  /// Undistort points： the first point is assummed as the base frame
  /// Compensate lidar points with IMU rotation (with only rotation now)
  if (imu_en) {
    cout << "Use IMU" << endl;
    UndistortPcl(meas, stat, *cur_pcl_un_);
  } else {
    cout << "No IMU, use constant velocity model" << endl;
    cov_acc = Eye3d * cov_acc_scale;
    cov_gyr = Eye3d * cov_gyr_scale;
    only_propag(meas, stat, cur_pcl_un_);
  }
}