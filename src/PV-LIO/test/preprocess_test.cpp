#include "preprocess.h"
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>

namespace {
std::shared_ptr<sensor_msgs::msg::PointCloud2> make_scan(bool with_time, bool zero_time = false) {
  pcl::PointCloud<velodyne_ros::Point> points;
  for (int i = 0; i < 12; ++i) {
    const double angle = -i * 30.0 * M_PI / 180.0;
    velodyne_ros::Point p;
    p.x = 10.0 * std::cos(angle); p.y = 10.0 * std::sin(angle); p.z = 1.0;
    p.intensity = 30.0f; p.ring = 0; p.time = zero_time ? 0.0f : i / 120.0f;
    points.push_back(p);
  }
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(points, *msg);
  if (!with_time) {
    // Keep the binary layout but omit the field, as in XYZIR bag recordings.
    auto &fields = msg->fields;
    fields.erase(std::remove_if(fields.begin(), fields.end(),
        [](const sensor_msgs::msg::PointField &f) { return f.name == "time"; }), fields.end());
  }
  return msg;
}
Preprocess configured(bool features = false) {
  Preprocess pre;
  pre.set(features, VELO16, 0.1, 1);
  pre.N_SCANS = 32;
  pre.SCAN_RATE = 10;
  return pre;
}
void expect_time_order(const PointCloudXYZI &points) {
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_TRUE(std::isfinite(points[i].curvature));
    EXPECT_GE(points[i].curvature, 0.0f);
    EXPECT_LE(points[i].curvature, 110.0f);
    if (i) EXPECT_LE(points[i - 1].curvature, points[i].curvature);
  }
}
}  // namespace

TEST(VelodynePreprocess, MissingTimeReconstructsInsteadOfDroppingEveryPoint) {
  auto pre = configured();
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(make_scan(false), result);
  ASSERT_EQ(result->size(), 11u);
  EXPECT_FALSE(pre.given_offset_time);
  EXPECT_NEAR(result->back().curvature, 91.6667, 0.01);
  expect_time_order(*result);
}

TEST(VelodynePreprocess, AllZeroTimeFieldUsesTheSameFallback) {
  auto pre = configured();
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(make_scan(true, true), result);
  ASSERT_EQ(result->size(), 11u);
  EXPECT_FALSE(pre.given_offset_time);
  EXPECT_NEAR(result->back().curvature, 91.6667, 0.01);
}

TEST(VelodynePreprocess, ProvidedSecondsArePreservedAsMilliseconds) {
  auto pre = configured();
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(make_scan(true), result);
  ASSERT_EQ(result->size(), 11u);
  EXPECT_TRUE(pre.given_offset_time);
  EXPECT_NEAR(result->back().curvature, 91.6667, 0.01);
  expect_time_order(*result);
}

TEST(VelodynePreprocess, EmptyScanIsSafe) {
  auto pre = configured();
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(std::make_shared<sensor_msgs::msg::PointCloud2>(), result);
  EXPECT_TRUE(result->empty());
}

TEST(VelodynePreprocess, MissingRingIsRejected) {
  auto pre = configured();
  auto msg = make_scan(false);
  auto &fields = msg->fields;
  fields.erase(std::remove_if(fields.begin(), fields.end(),
      [](const sensor_msgs::msg::PointField &f) { return f.name == "ring"; }), fields.end());
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(msg, result);
  EXPECT_TRUE(result->empty());
}

TEST(VelodynePreprocess, InvalidRingsNeverIndexPastConfiguredScanLines) {
  auto pre = configured();
  auto msg = make_scan(false);
  pcl::PointCloud<velodyne_ros::PointWithoutTime> points;
  pcl::fromROSMsg(*msg, points);
  for (auto &p : points) p.ring = 32;
  pcl::toROSMsg(points, *msg);
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(msg, result);
  EXPECT_TRUE(result->empty());
}

TEST(VelodynePreprocess, FeatureExtractionUsesMillisecondsForProvidedTime) {
  auto pre = configured(true);
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(make_scan(true), result);
  ASSERT_EQ(pre.pl_buff[0].size(), 12u);
  EXPECT_NEAR(pre.pl_buff[0].back().curvature, 91.6667, 0.01);
}

TEST(VelodynePreprocess, RecordedXYZIRScanRetainsPointsAndHasAValidDuration) {
  const char *path = std::getenv("PV_LIO_TEST_CLOUD_CDR");
  if (!path) GTEST_SKIP() << "Set PV_LIO_TEST_CLOUD_CDR to a serialized PointCloud2 sample";
  std::ifstream stream(path, std::ios::binary);
  ASSERT_TRUE(stream.good());
  const std::vector<char> data((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  rclcpp::SerializedMessage serialized(data.size());
  auto &raw = serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer, data.data(), data.size());
  raw.buffer_length = data.size();
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  serialization.deserialize_message(&serialized, msg.get());
  auto pre = configured();
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  pre.process(msg, result);
  ASSERT_GT(result->size(), msg->width * msg->height / 2);
  ASSERT_FALSE(result->empty());
  expect_time_order(*result);
  EXPECT_GT(result->back().curvature, 90.0f);
  std::cout << "Recorded scan: " << msg->width * msg->height << " input, " << result->size()
            << " retained, duration " << result->back().curvature << " ms" << std::endl;
}
