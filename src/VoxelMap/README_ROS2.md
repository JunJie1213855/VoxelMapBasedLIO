# ROS 2 Humble port

This package now uses ament_cmake, rclcpp, tf2_ros and livox_ros_driver2.
See the workspace README for sensor configuration and algorithm details.

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
cd /home/ros/rosws/voxelmap_lio_ws
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 1
source install/setup.bash
ros2 launch voxel_map mapping_velodyne.launch.py rviz:=false
```

Launch arguments: rviz (true), use_sim_time (false), params_file (sensor YAML).

## UrbanNav HDL-32E bag

This preset runs the original VoxelMap LiDAR-only estimator (`imu.imu_en: false`),
not the PV-LIO estimator. The bag has XYZIR points without a `time` field; the
Velodyne handler reads `ring` directly and does not apply the HDL-64E angle filter.
No IMU deskew is performed in this preset.

After building and sourcing the workspace, terminal 1:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
ros2 launch voxel_map mapping_urbannav.launch.py use_sim_time:=true
```

Terminal 2, with the same sourced environment and RMW:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
ros2 bag play /mnt/d/dataset/SLAM/UrbanNav/2020-03-14-16-45-35 \
  --clock --rate 0.5 --read-ahead-queue-size 20 --topics /velodyne_points
```

Start with playback at 0.5x; increase to 1x if processing keeps up.
The UrbanNav preset now enables all three RViz displays in `camera_init`:

- `/cloud_registered`: the current scan.
- `/Laser_map`: an accumulated, voxel-downsampled global point cloud for display.
- `/planes`: plane markers through voxel layer 4.

Global map and planes publish in background workers at a default 2-second period,
with limits of 100000 points and 5000 planes. The display cloud automatically
coarsens if it exceeds the limit, preserving previously visited regions. It is
separate from the estimator's probabilistic voxel map. Busy visualization workers
skip stale display updates; the estimator never waits for serialization or DDS.
Plane snapshots copy values before crossing threads, clear stale markers and use
positive marker dimensions even for near-zero plane thickness. Cached snapshots
repeat after input stops, allowing RViz to reconnect.

Parameters are under `visualization`: `pub_global_map`, `pub_voxel_map`,
`publish_max_voxel_layer`, `global_map_voxel_size`, `map_publish_period`,
`max_global_map_points`, `max_voxel_planes`. Other sensor presets retain their
existing enable flags. RViz uses Best Effort with Transient Local for `/Laser_map`
and `/planes`, and Best Effort/Volatile for the current scan and trajectory.
Restart the launch and RViz after updating to load these QoS settings.
Do not run both estimators in the same ROS domain: several topic/TF names overlap.

Validation:

```bash
colcon test --packages-select voxel_map
colcon test-result --test-result-base build/voxel_map --verbose
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 tests/ros2_smoke_test.py
```

## RoboSense Airy 96: aligned with FAST_LIO_ROBOAIRY

The Airy profile follows the user's reference at
`/home/ros/rosws/LIO_Nav2_ROS2/src/localization/lio/FAST_LIO_ROBOAIRY`.
VoxelMap retains its own probabilistic voxel estimator; this is not a replacement
of that estimator with FAST-LIO's ikd-tree filter.

The preset now uses **LiDAR + built-in IMU** by default:

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
source /home/ros/rosws/voxelmap_lio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
ros2 launch voxel_map mapping_robosenseairy.launch.py
```

For a bag, start that launch with `use_sim_time:=true`, then use the same sourced
environment in a second terminal. **Both topics must be played**:

```bash
ros2 bag play /mnt/d/dataset/SLAM/robosense/mapping_v2 \
  --clock --rate 0.5 --read-ahead-queue-size 200 \
  --topics /rslidar_points /rslidar_imu_data
```

Launch overrides: `input_topic`, `imu_topic`, `use_imu`, `rviz`, `use_sim_time`,
`params_file`. Use `use_imu:=false` to run without IMU; that mode keeps the
original LiDAR coordinate convention. Raw IMU messages are published by the driver
or bag player, not re-published by VoxelMap.

### Point preprocessing

- `lidar_type: 5`, `scan_line: 96`, `timestamp_unit: 0` (seconds), blind zone 0.5 m,
  maximum range 150 m, and `point_filter_num: 3`, matching the reference profile.
- Organized Airy clouds (`width=96`, `height=time columns`) are decimated by time
  column, preserving all channels. Transposed organized clouds and unorganized
  per-ring streams also preserve each channel when decimating.
- Float/integer intensity layouts, unaligned field offsets, padded rows and both
  byte orders are supported without a separate Velodyne converter.
- Absolute point timestamps define scan start; offsets are stored as milliseconds
  and sorted. Scan end is computed before sampling/range filtering, so losing the
  last sampled column does not shorten IMU synchronization.
- Point timestamps are authoritative even when the header clock differs, as in the
  reference. Candidates must be within two nominal scan periods of the first
  valid point timestamp; relative offsets use zero as their origin. IMU and point
  timestamps must use the same clock.
- Missing `ring` or time fields triggers the reference's XYZI/zero-time fallback.
  This can run with `use_imu:=false`; IMU mode rejects clouds without usable timing.
- NaN returns, invalid rings and out-of-range returns are discarded. Malformed or
  empty clouds do not enter the estimator.

Parameters under `preprocess`: `max_range`, `scan_rate`, `timestamp_unit`
(seconds=0, milliseconds=1, microseconds=2, nanoseconds=3), and
`timestamp_is_absolute` (true for `timestamp`; `time` is a relative alias).

### IMU synchronization, gravity and extrinsics

The copied LiDAR-to-IMU extrinsic is:

```yaml
imu:
  imu_en: true
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [0.0, -1.0, 0.0,
                -1.0, 0.0, 0.0,
                 0.0, 0.0, -1.0]
  gravity_ref: [-0.0205, -0.0912, -9.8684]
```

The profile uses the reference's 0.1 accelerometer/gyro noise covariances and
0.0001 bias covariances. Initialization estimates gyro bias and accelerometer
scale from stationary measurements, aligns measured gravity to the configured
reference direction and normalizes gravity magnitude to 9.81 m/s^2.
Keep the device stationary during the initial approximately one second.

Synchronization retains an IMU sample on each side of scan boundaries, including
the final partial interval. Prediction starts at the previous scan end rather
than re-integrating overlapping samples or dropping gaps. Deskew applies both
rotation and translation of the extrinsic exactly once. VoxelMap's measurement
model receives points in the scan-end IMU body frame, and its range-noise
covariances are rotated consistently into that frame. Odometry, TF and visual
snapshots use scan-end timestamps in IMU mode.

`common.time_offset_lidar_to_imu` follows the reference's convention:
`corrected_imu_stamp = input_imu_stamp - offset`. Default is zero. The sensor IMU
subscription has depth 4000; timestamp gaps above 50 ms generate a diagnostic.
Global maps and planes retain background publishing and bounded payloads.

The reference's split-scan RSM1_BREAK and FAST-LIO feature extraction are not
needed by its selected `lidar_type: 5`/single-frame profile and are not added here.
The existing UrbanNav preset remains LiDAR-only.

Optional recorded-cloud tests:

```bash
VOXELMAP_AIRY_TEST_CDR=/tmp/airy_mapping_first_cloud.cdr \
  colcon test --packages-select voxel_map
```
