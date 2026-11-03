#include <gtest/gtest.h>
class ImuProcess;
#include "voxel_map_util.hpp"
#include "async_map_publisher.hpp"

std::vector<pointWithCov> samples(const std::vector<Eigen::Vector3d> &positions) {
  std::vector<pointWithCov> result;
  for (const auto &p : positions) {
    pointWithCov v;
    v.point = p;
    v.cov = Eigen::Matrix3d::Identity() * .001;
    result.push_back(v);
  }
  return result;
}
TEST(PlaneFit, RepeatedPointsDoNotCrashOrBecomePlane) {
  OctoTree tree(0, 0, {5}, 100, 100, .01f);
  Plane plane;
  auto points = samples(std::vector<Eigen::Vector3d>(6, Eigen::Vector3d(1, 2, 3)));
  tree.init_plane(points, &plane);
  EXPECT_FALSE(plane.is_plane);
  EXPECT_TRUE(plane.plane_cov.allFinite());
  tree.update_plane(points, &plane);
  EXPECT_FALSE(plane.is_plane);
  EXPECT_TRUE(plane.normal.allFinite());
  delete tree.plane_ptr_;
}
TEST(PlaneFit, CollinearPointsHaveNoUniquePlaneNormal) {
  OctoTree tree(0, 0, {5}, 100, 100, .01f);
  Plane plane;
  tree.init_plane(samples({{0,0,0},{1,0,0},{2,0,0},{3,0,0},{4,0,0}}), &plane);
  EXPECT_FALSE(plane.is_plane);
  EXPECT_TRUE(plane.plane_cov.allFinite());
  delete tree.plane_ptr_;
}
TEST(PlaneFit, PlanarGridHasFiniteCovarianceAndNormal) {
  OctoTree tree(0, 0, {5}, 100, 100, .01f);
  Plane plane;
  auto points = samples({{0,0,1},{1,0,1},{0,1,1},{1,1,1},{2,1,1},{1,2,1}});
  tree.init_plane(points, &plane);
  ASSERT_TRUE(plane.is_plane);
  EXPECT_TRUE(plane.plane_cov.allFinite());
  EXPECT_NEAR(std::abs(plane.normal.z()), 1., 1e-8);
  tree.update_plane(points, &plane);
  EXPECT_TRUE(plane.is_plane);
  EXPECT_TRUE(plane.normal.allFinite());
  delete tree.plane_ptr_;
}

TEST(PointCovariance, HorizontalRayIsFiniteAndHasExpectedVariance) {
  Eigen::Vector3d point(10, 0, 0);
  Eigen::Matrix3d cov;
  calcBodyCov(point, .04f, .1f, cov);
  ASSERT_TRUE(cov.allFinite());
  EXPECT_NEAR(cov(0,0), .04*.04, 1e-9);
  const double tangent_var = 100 * std::pow(std::sin(DEG2RAD(.1f)), 2);
  EXPECT_NEAR(cov(1,1), tangent_var, 1e-9);
  EXPECT_NEAR(cov(2,2), tangent_var, 1e-9);
  EXPECT_NEAR(cov(0,1), 0., 1e-12);
}
TEST(PointCovariance, GeneralRayMatchesOrthonormalTangentConstruction) {
  Eigen::Vector3d point(3,4,5);
  Eigen::Vector3d direction = point.normalized();
  Eigen::Vector3d tangent1(1,1,-(direction.x()+direction.y())/direction.z());
  tangent1.normalize();
  Eigen::Vector3d tangent2 = tangent1.cross(direction).normalized();
  Eigen::Matrix3d cross;
  cross << 0,-direction.z(),direction.y(),direction.z(),0,-direction.x(),-direction.y(),direction.x(),0;
  Eigen::Matrix<double,3,2> basis;
  basis.col(0)=tangent1; basis.col(1)=tangent2;
  auto angular = point.norm()*cross*basis;
  Eigen::Matrix3d expected = double(.04f)*.04f*direction*direction.transpose() +
      std::pow(std::sin(DEG2RAD(.1f)),2)*angular*angular.transpose();
  Eigen::Matrix3d cov;
  calcBodyCov(point,.04f,.1f,cov);
  EXPECT_TRUE(cov.isApprox(expected,1e-10));
}

TEST(PlaneDisplay, SnapshotIncludesUnchangedPlanesAndCopiesTheirValues) {
  OctoTree tree(0, 0, {5}, 100, 100, .01f);
  tree.plane_ptr_->is_plane = true;
  tree.plane_ptr_->is_update = false;
  tree.plane_ptr_->center = Eigen::Vector3d(1,2,3);
  std::vector<Plane> snapshot;
  voxelmap_display::collect_planes(&tree, 0, snapshot);
  ASSERT_EQ(snapshot.size(), 1u);
  tree.plane_ptr_->center.x() = 10;
  EXPECT_DOUBLE_EQ(snapshot.front().center.x(), 1.);
  delete tree.plane_ptr_;
}
TEST(PlaneDisplay, MarkerDimensionsStayFiniteAndVisibleForThinPlanes) {
  Plane plane;
  plane.center.setZero();
  plane.normal = Eigen::Vector3d::UnitZ();
  plane.x_normal = Eigen::Vector3d::UnitX();
  plane.y_normal = Eigen::Vector3d::UnitY();
  plane.id = 1;
  plane.min_eigen_value = -1e-9f;
  plane.mid_eigen_value = 0;
  plane.max_eigen_value = 1;
  visualization_msgs::msg::MarkerArray message;
  pubSinglePlane(message, "plane", plane, .8f, Eigen::Vector3d(1,0,0));
  ASSERT_EQ(message.markers.size(), 1u);
  for (double v : {message.markers[0].scale.x, message.markers[0].scale.y, message.markers[0].scale.z}) {
    EXPECT_TRUE(std::isfinite(v));
    EXPECT_GT(v, 0);
  }
}

TEST(PlaneDisplay, ReflectedEigenbasisBecomesUnitQuaternionWithCorrectNormal) {
  geometry_msgs::msg::Quaternion message;
  const Eigen::Vector3d normal = -Eigen::Vector3d::UnitZ();
  CalcVectQuation(Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),normal,message);
  Eigen::Quaterniond q(message.w,message.x,message.y,message.z);
  EXPECT_NEAR(q.norm(),1.,1e-12);
  EXPECT_TRUE((q.toRotationMatrix().col(2)).isApprox(normal,1e-12));
  EXPECT_NEAR(q.toRotationMatrix().determinant(),1.,1e-12);
}
