#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace voxelmap_ros2
{
  // Preserve the sensor timestamp when converting floating point scan times.
  inline builtin_interfaces::msg::Time time_from_seconds(double seconds)
  {
    return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
  }

  inline geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
  }
} // namespace voxelmap_ros2
