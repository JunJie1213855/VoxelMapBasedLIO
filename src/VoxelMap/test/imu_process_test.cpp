#include <gtest/gtest.h>
#include "IMU_Processing.hpp"

namespace {
sensor_msgs::msg::Imu::ConstSharedPtr sample(double time,const V3D &acc,const V3D &gyro) {
  auto msg=std::make_shared<sensor_msgs::msg::Imu>();
  msg->header.stamp=voxelmap_ros2::time_from_seconds(time);
  msg->linear_acceleration.x=acc.x();msg->linear_acceleration.y=acc.y();msg->linear_acceleration.z=acc.z();
  msg->angular_velocity.x=gyro.x();msg->angular_velocity.y=gyro.y();msg->angular_velocity.z=gyro.z();return msg;
}
MeasureGroup scan(double start,const V3D &acc=V3D(0,0,9.81),const V3D &gyro=V3D::Zero()) {
  MeasureGroup result;result.lidar_beg_time=start;result.lidar_end_time=start+.1;
  PointType first{};first.x=1;first.curvature=0;result.lidar->push_back(first);
  first.curvature=100;result.lidar->push_back(first);
  for(int i=0;i<=20;++i) result.imu.push_back(sample(start+i*.005,acc,gyro));
  result.imu.push_back(sample(start+.105,acc,gyro));return result;
}
double initialize(ImuProcess &imu,StatesGroup &state) {
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  for(int i=0;i<20;++i) {
    const double start=1000+i*.1;imu.Process(scan(start),state,output);
    if(imu.initialized()) return start+.1;
  }
  ADD_FAILURE()<<"IMU did not initialize";return 1002;
}
}
TEST(ImuDeskew, GravityAlignmentUsesConfiguredReference) {
  ImuProcess imu;StatesGroup state;
  const V3D reference(-.0205,-.0912,-9.8684);imu.set_gravity_ref(reference);
  initialize(imu,state);
  EXPECT_NEAR(state.gravity.norm(),9.81,1e-9);
  EXPECT_TRUE(state.gravity.normalized().isApprox(reference.normalized(),1e-9));
  EXPECT_TRUE((state.rot_end*V3D(0,0,-1)).isApprox(reference.normalized(),1e-9));
}
TEST(ImuDeskew, StationaryPointsReceiveFullExtrinsicExactlyOnce) {
  ImuProcess imu;StatesGroup state;
  M3D rotation;rotation<<0,-1,0,-1,0,0,0,0,-1;const V3D translation(.1,.2,.3);
  imu.set_extrinsic(translation,rotation);
  const double next=initialize(imu,state);auto data=scan(next);
  for(auto &p:*data.lidar) {p.x=1;p.y=2;p.z=3;}
  PointCloudXYZI::Ptr output(new PointCloudXYZI);imu.Process(data,state,output);
  ASSERT_EQ(output->size(),2u);
  const V3D expected=rotation*V3D(1,2,3)+translation;
  for(const auto &p:*output) EXPECT_TRUE(V3D(p.x,p.y,p.z).isApprox(expected,1e-6));
  EXPECT_NEAR(state.pos_end.norm(),0,1e-8);
}
TEST(ImuDeskew, AngularMeasurementsChangePoseAndCompensateEarlyPoints) {
  ImuProcess imu;StatesGroup state;const double next=initialize(imu,state);
  auto data=scan(next,V3D(0,0,9.81),V3D(0,0,1));
  // Remove the rate discontinuity at the first endpoint by initializing this
  // frame's retained start sample to zero; linear interpolation adds half a step.
  PointCloudXYZI::Ptr output(new PointCloudXYZI);imu.Process(data,state,output);
  ASSERT_EQ(output->size(),2u);
  const double angle=std::atan2(state.rot_end(1,0),state.rot_end(0,0));
  EXPECT_GT(angle,.095);EXPECT_LT(angle,.101);
  EXPECT_NEAR(output->front().x,std::cos(angle),1e-5);
  EXPECT_NEAR(output->front().y,-std::sin(angle),1e-5);
  EXPECT_NEAR(output->back().x,1,1e-5);EXPECT_NEAR(output->back().y,0,1e-5);
}
TEST(ImuDeskew, TranslationAndScanGapsPropagateFromPreviousEnd) {
  ImuProcess imu;StatesGroup state;const double next=initialize(imu,state);
  state.vel_end=V3D(1,0,0);auto data=scan(next);
  PointCloudXYZI::Ptr output(new PointCloudXYZI);imu.Process(data,state,output);
  ASSERT_EQ(output->size(),2u);EXPECT_NEAR(state.pos_end.x(),.1,1e-6);
  EXPECT_NEAR(output->front().x,.9,1e-6);EXPECT_NEAR(output->back().x,1.,1e-6);
  auto after_gap=scan(next+.2);imu.Process(after_gap,state,output);
  ASSERT_EQ(output->size(),2u);EXPECT_NEAR(state.pos_end.x(),.3,1e-6);
}
TEST(ImuDeskew, InsufficientCoverageDoesNotReusePreviousCloud) {
  ImuProcess imu;StatesGroup state;const double next=initialize(imu,state);
  PointCloudXYZI::Ptr output(new PointCloudXYZI);imu.Process(scan(next),state,output);
  ASSERT_FALSE(output->empty());auto missing=scan(next+.1);missing.imu.clear();
  imu.Process(missing,state,output);EXPECT_TRUE(output->empty());
}
