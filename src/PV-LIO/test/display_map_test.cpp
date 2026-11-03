#include "display_map.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace {
pcl::PointCloud<pcl::PointXYZI> points(std::initializer_list<float> xs) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (float x : xs) {
    pcl::PointXYZI point;
    point.x = x; point.y = 0.1f; point.z = 0.1f; point.intensity = 20.0f;
    cloud.push_back(point);
  }
  return cloud;
}
}

TEST(DisplayMap, KeepsEarlierRegionsWhenTheCurrentScanMovesAway) {
  pv_lio::DisplayMap map;
  map.add(points({1.1f, 1.2f}));
  EXPECT_EQ(map.size(), 1u);
  map.add(points({20.1f, 20.2f}));
  ASSERT_EQ(map.size(), 2u);
  bool old_region = false, current_region = false;
  for (const auto &point : map.cloud()) {
    old_region |= point.x < 2.0f;
    current_region |= point.x > 20.0f;
  }
  EXPECT_TRUE(old_region);
  EXPECT_TRUE(current_region);
}

TEST(DisplayMap, RepeatedScansDoNotGrowTheMapInAnAlreadyObservedCell) {
  pv_lio::DisplayMap map;
  for (int i = 0; i < 100; ++i) map.add(points({1.1f, 1.2f, 1.3f}));
  EXPECT_EQ(map.size(), 1u);
}

TEST(DisplayMap, OppositeSidesOfTheOriginOccupyDifferentVoxels) {
  pv_lio::DisplayMap map;
  map.add(points({-0.01f, 0.01f}));
  EXPECT_EQ(map.size(), 2u);
}

TEST(DisplayMap, RejectsInvalidCoordinatesAndVoxelSizes) {
  pv_lio::DisplayMap map;
  map.add(points({std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}));
  EXPECT_TRUE(map.empty());
  EXPECT_THROW(map.set_resolution(0.0), std::invalid_argument);
  EXPECT_THROW(map.set_resolution(-1.0), std::invalid_argument);
  EXPECT_THROW(map.set_resolution(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(DisplayMap, ResolutionControlsHowMuchHistoryIsDownsampled) {
  pv_lio::DisplayMap map;
  map.set_resolution(1.0);
  map.add(points({1.1f, 1.4f}));
  EXPECT_EQ(map.size(), 1u);
  map.set_resolution(0.1);
  EXPECT_TRUE(map.empty());
  map.add(points({1.1f, 1.4f}));
  EXPECT_EQ(map.size(), 2u);
}

TEST(DisplayMap, CoarseningBoundsPayloadWithoutRemovingPreviouslyVisitedRegions) {
  pv_lio::DisplayMap map;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int i = 0; i < 200; ++i) {
    pcl::PointXYZI point;
    point.x = i * 0.4f; point.y = point.z = 0.1f; point.intensity = 10.0f;
    cloud.push_back(point);
  }
  map.add(cloud);
  ASSERT_GT(map.size(), 16u);
  map.coarsen_to(16);
  EXPECT_LE(map.size(), 16u);
  bool beginning = false, end = false;
  for (const auto &point : map.cloud()) {
    beginning |= point.x < 8.0f;
    end |= point.x > 72.0f;
  }
  EXPECT_TRUE(beginning);
  EXPECT_TRUE(end);
}
