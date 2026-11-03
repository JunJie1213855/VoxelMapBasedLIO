# ROS 2 Humble port

This package now uses ament_cmake, rclcpp, tf2_ros and livox_ros_driver2.
See the workspace README for sensor configuration and algorithm details.

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
cd /home/ros/rosws/voxelmap_lio_ws
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 1
source install/setup.bash
ros2 launch pv_lio mapping_velodyne.launch.py rviz:=false
```

Launch arguments: rviz (true), use_sim_time (false), params_file (sensor YAML).

## Velodyne / UrbanNav clouds without point timestamps

The UrbanNav sequence `2020-03-14-16-45-35` contains `x, y, z, intensity, ring`
and no `time` field; ring IDs are 0--31. Use `preprocess.scan_line: 32` and
`preprocess.scan_rate: 10`. The preprocessor accepts XYZIR directly and estimates
relative point times from each ring's clockwise azimuth sweep. It emits one
warning explaining that motion compensation is approximate. All-zero time fields
use the same fallback. Native nonzero `time` values remain relative seconds.

The fallback is approximate and assumes an acquisition-ordered spinning scan;
recorded per-point timestamps are preferable when available. Missing `time` must
not be resolved by changing the configured laser line count.

Validated on 2026-11-03: the first recorded scan retained 64,776 of 64,808 points,
with estimated duration 100.133 ms. Eight preprocessing regression cases passed.
A short native bag replay at half speed initialized the map and produced 62
finite odometry messages, with no missing-field or empty-cloud warnings. This
checks data processing, not trajectory accuracy or the rig's extrinsic calibration.

The verified replay used Cyclone DDS with localhost-only mode disabled:

```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash
source /home/ros/rosws/voxelmap_lio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
ros2 launch pv_lio mapping_velodyne.launch.py use_sim_time:=true rviz:=false
# In a second terminal with the same environment:
ros2 bag play /mnt/d/dataset/SLAM/UrbanNav/2020-03-14-16-45-35 \
  --clock --topics /velodyne_points /imu/data
```

Run the C++ regression tests with `colcon test --packages-select pv_lio` and inspect
results using `colcon test-result --verbose`. The optional recorded-scan case reads
one serialized PointCloud2 from `PV_LIO_TEST_CLOUD_CDR`; it is skipped if unset.

## Global map and voxel plane visualization

The `velodyne.yaml` and `urbannav.yaml` presets now enable both map displays.
RViz shows three different data products in the fixed frame `camera_init`:

- **Current scan** (`/cloud_registered`): one registered scan; Decay Time stays 0.
- **Global map** (`/Laser_map`): a voxel-filtered accumulation of registered scans.
- **Voxel planes** (`/planes`): the estimator's adaptive plane map, including all
  configured octree layers in these two presets.

The accumulated point map is only a visualization history; it does not replace
or alter the estimator's probabilistic voxel map. The display starts with 0.5 m voxels and automatically coarsens its stored
history if it exceeds 100,000 points. This preserves coverage of earlier areas
while bounding message size. Plane rendering uniformly samples at most 5,000
valid planes across the eligible octree layers. These are display limits only.

Configure these publication parameters under `publish`:

```yaml
pub_global_map: true
global_map_voxel_size: 0.5   # metres; larger values produce a smaller display map
map_publish_period: 2.0     # seconds between display snapshots
max_global_map_points: 100000
max_voxel_planes: 5000
pub_voxel_map: true
publish_max_voxel_layer: 4  # use mapping.max_layer to include every map level
```

Both map topics publish complete snapshots with best-effort, transient-local
QoS and depth 1. RViz must use **Reliability Policy: Best Effort** and
**Durability Policy: Transient Local**. Single-frame clouds and `/path` also use
best-effort QoS and a bounded history; `/Odometry` retains reliable delivery.
The background worker repeats its cached latest snapshot while input is idle,
which also supports DDS implementations that do not deliver best-effort historical
data to late subscribers. The mapping node must remain running.

After updating, restart the mapping node and RViz to load the new binary,
parameters, and display configuration. These parameters are read at startup.
For UrbanNav use `ros2 launch pv_lio mapping_urbannav.launch.py use_sim_time:=true`.

## Preventing visualization load from disrupting localization

Global cloud accumulation, display-map coarsening, point-cloud transforms,
Marker generation and all large visual-topic publications now run in background
workers. The estimator hands off immutable point/pose copies and plane value
snapshots; workers never access its mutable filter state or octrees. Queues are
bounded and stale visualization work is skipped rather than blocking estimation.
Map visualization samples scans at `publish.map_publish_period`, so it may contain
fewer duplicate observations than a full-rate offline map export.

RViz defaults to 10 FPS. Its map, scan, plane and path displays must accept
best-effort delivery; using reliable subscriptions for these best-effort writers
will result in no messages until the display QoS is corrected or RViz reloads.

The UrbanNav bag preset sets `common.reliable_input: true` to avoid losing IMU
samples during DDS/CPU load spikes. The IMU receive history holds 4,000 samples.
For physical drivers that offer only best-effort sensor data, leave
`common.reliable_input: false` (the default for other presets).

Use the recorded-bag publisher QoS override in the playback terminal:

```bash
ros2 bag play /mnt/d/dataset/SLAM/UrbanNav/2020-03-14-16-45-35 \
  --clock --topics /velodyne_points /imu/data \
  --qos-profile-overrides-path /home/ros/rosws/voxelmap_lio_ws/src/PV-LIO/config/urbannav_bag_qos.yaml
```

`IMU timestamp gap ...` warnings indicate missing or out-of-order input timing;
`common.imu_gap_warn_threshold` defaults to 0.05 seconds and can be set to 0 to
disable that diagnostic. Visualization load must not be addressed by changing
IMU noise, calibration, or estimator voxel-map parameters.

The reproducible comparison test replays identical bag timestamps with display
publication off/on, records odometry, and checks per-timestamp pose differences:

```bash
python3 tests/compare_visualization.py --bag /path/to/ros2_bag --duration 60
```

Validation of the load-isolation fix on 2026-11-03:

- Slow-publisher tests confirm that blocked visualization work does not block
  the producer and that the pending queue retains only the latest request.
- The first 60 seconds of UrbanNav were replayed with map/plane publication off
  and on using the same calibration and estimator parameters. All 570 matching
  odometry timestamps had exactly equal positions; neither replay reported IMU
  gaps or lost point-to-plane matches.
- An additional 150-second replay with visualization enabled produced 1,497
  odometry messages, with no IMU-gap or `No Effective Points` warnings. The display
  stayed below its point/plane payload limits.
- These checks exercise message publication and estimator output without the
  RViz GUI renderer. They do not establish dataset trajectory accuracy.
