# VoxelMap / PV-LIO for ROS 2 Humble

This workspace ports both original ROS 1 packages to ROS 2 Humble on Ubuntu 22.04.
Package names and mapping algorithms are retained: `voxel_map` (LiDAR odometry)
and `pv_lio` (LiDAR + IMU odometry). They are independent nodes; launch one at a time.

## Dependencies and build

The source requires ROS 2 Humble, colcon, a C++14 compiler, Eigen 3, PCL and OpenMP.
Both packages use `livox_ros_driver2/msg/CustomMsg`, including when building for
non-Livox sensors. Build and source the ROS 2 Livox driver workspace first.
No ROS 1 installation or catkin is required. Unused direct dependencies on
Ceres, OpenCV, Python C API and eigen_conversions were removed; ROS interface
generation still uses the standard ROS 2 Python build tooling.

On this machine the Livox dependency is already built in
`/home/ros/rosws/livox_ros_ws`:

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
cd /home/ros/rosws/voxelmap_lio_ws
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Alternatively run `./build_ros2.sh`. Set `LIVOX_WS` to use another driver workspace.
The limited build parallelism avoids excessive memory use in the Eigen-heavy
PV-LIO translation unit. Runtime OpenMP parallelism defaults to four workers and
can be changed with `-DVOXELMAP_OMP_THREADS=4`.

## Run

```bash
# PV-LIO, Velodyne PointCloud2 + IMU
ros2 launch pv_lio mapping_velodyne.launch.py

# PV-LIO, Livox CustomMsg + IMU
ros2 launch pv_lio mapping_avia.launch.py

# PV-LIO, Robosense / Hesai with point-format converters
ros2 launch pv_lio mapping_robosense.launch.py input_topic:=/rslidar_points
ros2 launch pv_lio mapping_hesai.launch.py input_topic:=/hesai/pandar

# Original VoxelMap, pre-undistorted point clouds
ros2 launch voxel_map mapping_velodyne.launch.py
ros2 launch voxel_map mapping_l515.launch.py

# Original VoxelMap, UrbanNav HDL-32E (LiDAR-only, no IMU deskew)
ros2 launch voxel_map mapping_urbannav.launch.py use_sim_time:=true

# Native RoboSense Airy 96-line XYZIRT + built-in IMU
ros2 launch voxel_map mapping_robosenseairy.launch.py
```

All launch files accept `rviz:=false`, `use_sim_time:=true`, and
`params_file:=/absolute/path/to/params.yaml`. For ROS 2 bag playback:

```bash
ros2 launch pv_lio mapping_velodyne.launch.py rviz:=false use_sim_time:=true
# In another terminal with the same environment:
ros2 bag play /path/to/ros2_bag --clock
```

`mapping_avia.launch.py` loads `avia_outdoor.yaml`; `mapping_horizon.launch.py`
loads `avia_indoor.yaml`. The horizon entry uses the Livox CustomMsg input without
unrelated point-format conversion nodes.

## Parameters and data contracts

Every sensor YAML now uses the ROS 2 `/**: ros__parameters:` structure.
Nested keys become dotted parameter names, for example `common.lid_topic`,
`mapping.voxel_size`, and `preprocess.scan_line`. Override the sensor configuration
before using real data; the included extrinsics are examples for particular rigs.
`extrinsic_T` and `extrinsic_R` transform LiDAR coordinates into the IMU frame.
Double arrays must contain floating point numbers; layer_point_size is an integer array.

PV-LIO inputs are a sensor-specific cloud and `sensor_msgs/msg/Imu`. Livox uses
`livox_ros_driver2/msg/CustomMsg`; Velodyne uses PointCloud2 fields x/y/z,
intensity, ring and time (seconds relative to scan start); Ouster uses its t field.
Velodyne XYZIR clouds without `time`, or with all-zero point times, use an
azimuth-based relative-time estimate. Timestamp-range filtering applies to real
point times rather than dropping every zero-time point before reconstruction.
True per-point times and accurate LiDAR/IMU synchronization are preferable.
Sensor subscribers use SensorDataQoS by default. The UrbanNav bag preset uses
reliable sensor inputs, with a larger IMU history and a bag QoS override file.
Large display topics use bounded best-effort delivery; odometry stays reliable.

PV-LIO publishes `/Odometry`, `/path`, `/cloud_registered`,
`/cloud_registered_body`, optional accumulated `/Laser_map`, and optional `/planes`.
The Velodyne and UrbanNav presets enable the global map and all plane-map layers.
Global snapshots start with 0.5 m voxels and use a 2 second publication period.
Rendering is capped at 100,000 global points (adaptive coarsening) and 5,000 planes.
The map topics use best-effort, transient-local QoS and background publication so
slow visualization consumers do not block the estimator.
See the package ROS 2 README for the map publication parameters.
TF is `world -> camera_init -> body`. VoxelMap publishes `/aft_mapped_to_init`,
`/path`, `/cloud_registered` (when enabled), `/cloud_effected`, and optional `/planes`
and `/Laser_map`; its TF is `camera_init -> aft_mapped`.
VoxelMap's UrbanNav preset enables the global map and plane layers 0 through 4.
Its display options are under `visualization`, including `pub_global_map`,
`pub_voxel_map`, `map_publish_period`, `max_global_map_points`, and `max_voxel_planes`.
Global snapshots and current-scan conversion publish in independent workers with
bounded queues and best-effort QoS; map/plane snapshots also repeat for late RViz
subscribers. Limits match PV-LIO's 100000 points/5000 planes and 2 second period.
Restart the launch and RViz to load the updated displays and QoS settings.
RViz 2 configurations are included.

The converters accept ROS 2 parameters `input_type` (XYZI or XYZIRT), `output_type`
(XYZI, XYZIR or XYZIRT), and, for Robosense, `use_frame_time` (default false).
Input topics and `/velodyne_points` can be remapped using standard ROS 2 remappings.

The original VoxelMap sensor presets remain LiDAR-only except Airy, whose profile
now uses IMU synchronization, gravity alignment, full extrinsics and deskew based
on the user's FAST_LIO_ROBOAIRY reference. See its package ROS 2 README. PV-LIO adds
IMU prediction, motion compensation and optional online extrinsic estimation.
Neither package adds loop closure or global graph optimization. The upstream
PV-LIO `pcd_save` settings remain placeholders; this migration does not add PCD export.

## Validation

Run the integration checks after sourcing `install/setup.bash`:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 tests/ros2_smoke_test.py
```

Validated on this machine on 2026-11-03:

- `colcon build`: both packages built and installed successfully.
- Eight launch variants loaded parameters and answered parameter services with
  RViz disabled and simulated time enabled.
- Small synthetic point clouds produced finite odometry, TF, cloud and plane outputs
  from both mapping nodes; PV-LIO also consumed synthetic IMU measurements.
- Both sensor converters preserved ring values and relative point timestamps.
- The four integration checks passed with Cyclone DDS (`rmw_cyclonedds_cpp`).

The integration checks use small clouds. Earlier localhost-only tests
(`ROS_LOCALHOST_ONLY=1`) failed to deliver large samples even to an independent
minimal ROS 2 C++ subscriber; some default Fast DDS checks also failed. With
`ROS_LOCALHOST_ONLY` unset and Cyclone DDS selected, a short native UrbanNav replay
successfully initialized the map and produced 62 finite odometry messages.

The UrbanNav sequence `2020-03-14-16-45-35` has 32 rings and no `time` field. Its
first 64,808-point scan now retains 64,776 points using the azimuth timing fallback
(100.133 ms estimated duration). Eight preprocessing regression cases passed,
including the recorded scan. See `src/PV-LIO/README_ROS2.md` for the replay command.
This verifies a short replay, not complete-sequence accuracy, sensor calibration,
or RViz GUI rendering.

The original READMEs retain upstream background and ROS 1 usage for historical
reference; use this README and `README_ROS2.md` for the migrated packages.
