#!/usr/bin/env python3
"""Compare estimator output with map visualization off/on using the same bag."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def run_case(args):
    os.environ["ROS_DOMAIN_ID"] = str(94 if args.mode == "off" else 95)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    import rclpy
    import rosbag2_py
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import MarkerArray
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/velodyne_points"]))
    _, data, _ = reader.read_next()
    first = deserialize_message(data, PointCloud2).header.stamp
    first_ns = first.sec * 1000000000 + first.nanosec
    del reader, data
    target = first_ns + round(args.duration * 1e9)
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    params = yaml.safe_load((Path(get_package_share_directory("pv_lio")) / "config/urbannav.yaml").read_text())
    publish = params["/**"]["ros__parameters"]["publish"]
    publish["pub_global_map"] = publish["pub_voxel_map"] = args.mode == "on"
    params_path = out / (args.mode + ".yaml")
    params_path.write_text(yaml.safe_dump(params, sort_keys=False))
    rclpy.init()
    node = rclpy.create_node("visualization_comparison_" + args.mode)
    poses = {}; map_counts = []; plane_counts = []; input_imus = []; cloud_counts = []
    def odometry(m):
        t = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
        p, q = m.pose.pose.position, m.pose.pose.orientation
        poses[t] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    def imu(m):
        input_imus.append(m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [node.create_subscription(Odometry, "/Odometry", odometry, 10000),
                     node.create_subscription(Imu, "/imu/data", imu, qos_profile_sensor_data),
                     node.create_subscription(PointCloud2, "/cloud_registered", lambda m: cloud_counts.append(m.width), qos_profile_sensor_data),
                     node.create_subscription(PointCloud2, "/Laser_map", lambda m: map_counts.append(m.width), qos),
                     node.create_subscription(MarkerArray, "/planes", lambda m: plane_counts.append(len(m.markers)), qos)]
    launch = bag = None
    log_path = Path("/home/ros/rosws/voxelmap_lio_ws/src/PV-LIO/Log/imu.txt")
    saved_log = log_path.read_bytes() if log_path.exists() else None
    try:
        with (out / (args.mode + "_node.log")).open("w") as log, (out / (args.mode + "_bag.log")).open("w") as bag_log:
            launch = subprocess.Popen(["ros2", "launch", "pv_lio", "mapping_urbannav.launch.py", "rviz:=false", "use_sim_time:=true", "params_file:=" + str(params_path)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 10
            while node.count_publishers("/Odometry") == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert node.count_publishers("/Odometry") > 0
            bag = subprocess.Popen(["ros2", "bag", "play", args.bag, "--clock", "--delay", "3", "--disable-keyboard-controls", "--read-ahead-queue-size", "2000", "--rate", str(args.rate), "--qos-profile-overrides-path", str(Path(get_package_share_directory("pv_lio")) / "config/urbannav_bag_qos.yaml"), "--topics", "/velodyne_points", "/imu/data"], stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.duration / args.rate * 2 + 20
            report = time.monotonic() + 15
            while time.monotonic() < deadline and (not poses or max(poses) < target):
                rclpy.spin_once(node, timeout_sec=.02)
                assert launch.poll() is None, "Mapping node exited"
                if time.monotonic() >= report:
                    elapsed = (max(poses) - first_ns) / 1e9 if poses else 0
                    print(args.mode, "bag seconds", round(elapsed, 1), "poses", len(poses), flush=True)
                    report += 15
            bag.send_signal(signal.SIGINT)
            try: bag.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL); bag.wait(timeout=3)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.02)
    finally:
        for proc in [bag, launch]:
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try: proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL); proc.wait(timeout=3)
        node.destroy_node(); rclpy.shutdown()
        if saved_log is None:
            if log_path.exists(): log_path.unlink()
        else: log_path.write_bytes(saved_log)
    text = (out / (args.mode + "_node.log")).read_text()
    selected = {str(t): p for t, p in poses.items() if first_ns + 3000000000 <= t <= target}
    result = {"poses": selected, "pose_count": len(poses), "cloud_count": len(cloud_counts),
              "observer_imu_messages": len(input_imus), "map_snapshots": len(map_counts), "plane_snapshots": len(plane_counts),
              "max_map_points": max(map_counts, default=0), "no_effective_points": text.count("No Effective Points"),
              "imu_gap_warnings": text.count("IMU timestamp gap"),
              "processed_bag_seconds": (max(poses) - first_ns) / 1e9 if poses else 0}
    (out / (args.mode + ".json")).write_text(json.dumps(result))
    print(args.mode, {k: v for k, v in result.items() if k != "poses"}, flush=True)
    assert result["processed_bag_seconds"] >= args.duration - .2
    assert selected and all(math.isfinite(v) for p in selected.values() for v in p)


def compare(args):
    for mode in ["off", "on"]:
        subprocess.run([sys.executable, __file__, "--bag", args.bag, "--duration", str(args.duration), "--rate", str(args.rate), "--output", args.output, "--mode", mode], check=True)
    out = Path(args.output)
    off, on = [json.loads((out / (mode + ".json")).read_text()) for mode in ["off", "on"]]
    common = sorted(set(off["poses"]) & set(on["poses"]))
    assert len(common) > .9 * min(len(off["poses"]), len(on["poses"]))
    distances, angles = [], []
    for stamp in common:
        a, b = off["poses"][stamp], on["poses"][stamp]
        distances.append(math.sqrt(sum((x-y)**2 for x,y in zip(a[:3], b[:3]))))
        norm = math.sqrt(sum(x*x for x in a[3:]) * sum(x*x for x in b[3:]))
        assert norm > 0
        dot = abs(sum(x*y for x,y in zip(a[3:], b[3:]))) / norm
        angles.append(2 * math.acos(min(1., max(0., dot))) * 180 / math.pi)
    summary = {"matched_poses":len(common), "max_position_difference_m": max(distances),
               "rms_position_difference_m": math.sqrt(sum(d*d for d in distances)/len(distances)),
               "max_orientation_difference_deg":max(angles),
               "off_matching_failures":off["no_effective_points"], "on_matching_failures":on["no_effective_points"]}
    (out / "comparison.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)
    assert summary["max_position_difference_m"] < .05
    assert summary["max_orientation_difference_deg"] < .1
    assert on["no_effective_points"] == 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--duration", type=float, default=150.)
    parser.add_argument("--rate", type=float, default=2.)
    parser.add_argument("--output", default="/tmp/pv_lio_visualization_comparison")
    parser.add_argument("--mode", choices=["off", "on"])
    args = parser.parse_args()
    if args.mode: run_case(args)
    else: compare(args)
