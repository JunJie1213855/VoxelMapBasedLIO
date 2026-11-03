#!/usr/bin/env python3
"""ROS 2 integration checks; source install/setup.bash before running."""
import contextlib
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

# Isolate test traffic from any robot or other workspace running on this host.
os.environ["ROS_DOMAIN_ID"] = "87"
# Large samples are not delivered reliably on this WSL loopback interface.
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
import yaml


class Ros2PortTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("voxelmap_port_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if predicate():
                return True
        return False

    @contextlib.contextmanager
    def process(self, command):
        with tempfile.TemporaryFile(mode="w+") as log:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                yield proc
                self.assertIsNone(proc.poll(), "Node/launch exited before shutdown")
            except Exception:
                log.seek(0)
                print(log.read()[-8000:], flush=True)
                raise
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT) if "launch" in command else os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=3)
                log.seek(0)
                output = log.read()
                self.assertNotIn("process has died", output, output[-6000:])
                self.assertNotIn("terminate called", output, output[-6000:])

    def test_01_launch_matrix_and_parameters(self):
        cases = [("voxel_map", "velodyne", "/voxel_mapping_odom", 64),
                 ("voxel_map", "l515", "/voxel_mapping_odom", 64),
                 ("voxel_map", "urbannav", "/voxel_mapping_odom", 32),
                 ("voxel_map", "robosenseairy", "/voxel_mapping_odom", 96),
                 ("pv_lio", "avia", "/pv_lio_node", 6),
                 ("pv_lio", "horizon", "/pv_lio_node", 6),
                 ("pv_lio", "velodyne", "/pv_lio_node", 32),
                 ("pv_lio", "robosense", "/pv_lio_node", 16),
                 ("pv_lio", "hesai", "/pv_lio_node", 32),
                 ("pv_lio", "urbannav", "/pv_lio_node", 32)]
        for package, sensor, name, lines in cases:
            with self.subTest(package=package, sensor=sensor):
                with self.process(["ros2", "launch", package, f"mapping_{sensor}.launch.py", "rviz:=false", "use_sim_time:=true"]):
                    client = self.node.create_client(GetParameters, name + "/get_parameters")
                    try:
                        self.assertTrue(client.wait_for_service(timeout_sec=8))
                        request = GetParameters.Request(names=["preprocess.scan_line", "use_sim_time"])
                        future = client.call_async(request)
                        self.assertTrue(self.wait_for(future.done))
                        self.assertEqual(future.result().values[0].integer_value, lines)
                        self.assertTrue(future.result().values[1].bool_value)
                    finally:
                        self.node.destroy_client(client)

    def pipeline(self, package, sensor="velodyne"):
        share = Path(get_package_share_directory(package))
        params = yaml.safe_load((share / ("config/" + sensor + ".yaml")).read_text())
        values = params["/**"]["ros__parameters"]
        values["common"]["lid_topic"] = "/port_test/points"
        values["common"]["imu_topic"] = "/port_test/imu"
        values["mapping"]["voxel_size"] = 2.0
        values["mapping"]["down_sample_size"] = 0.1
        values["preprocess"]["blind"] = 0.1
        values["preprocess"]["calib_laser"] = False
        if sensor == "robosenseairy":
            values["preprocess"]["point_filter_num"] = 1
        extrinsics = values["imu"] if package == "voxel_map" else values["mapping"]
        extrinsics["extrinsic_T"] = [0.0, 0.0, 0.0]
        extrinsics["extrinsic_R"] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        if package == "pv_lio":
            values["mapping"]["extrinsic_est_en"] = False
            values["publish"]["pub_voxel_map"] = True
            values["publish"]["pub_global_map"] = True
            values["publish"]["map_publish_period"] = 0.2
        else:
            values["visualization"]["pub_voxel_map"] = True
            values["visualization"]["pub_point_cloud"] = True
            values["visualization"]["pub_global_map"] = True
            values["visualization"]["map_publish_period"] = 0.2
        odometry, clouds, transforms, planes = [], [], [], []
        global_maps = []
        snapshot_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = "/Odometry" if package == "pv_lio" else "/aft_mapped_to_init"
        subscriptions = [self.node.create_subscription(Odometry, topic, odometry.append, 10),
                         self.node.create_subscription(PointCloud2, "/cloud_registered", clouds.append, qos_profile_sensor_data),
                         self.node.create_subscription(TFMessage, "/tf", transforms.append, 10),
                         self.node.create_subscription(MarkerArray, "/planes", planes.append, qos_profile_sensor_data)]
        subscriptions.append(self.node.create_subscription(PointCloud2, "/Laser_map", global_maps.append, snapshot_qos))
        cloud_pub = self.node.create_publisher(PointCloud2, "/port_test/points", qos_profile_sensor_data)
        imu_pub = self.node.create_publisher(Imu, "/port_test/imu", qos_profile_sensor_data)
        clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        fields = [PointField(name=n, offset=off, datatype=typ, count=1) for n, off, typ in
                  [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("timestamp" if sensor == "robosenseairy" else "time", 18, 8 if sensor == "robosenseairy" else 7)]]
        points = []
        for i in range(8):
            for j in range(8):
                jitter = 0.001 * math.sin(i * 7 + j)
                points.extend([(4.1 + i * .4, -.9 + j * .3, -1.1 + jitter, 30.0, i % 16, 0.0),
                               (7.1 + jitter, -.9 + j * .3, -.2 - i * .4, 30.0, i % 16, 0.0),
                               (4.1 + j * .3, 3.1 + jitter, -.2 - i * .4, 30.0, i % 16, 0.0)])
        points = [(*p[:-1], index * .09 / (len(points) - 1)) for index, p in enumerate(points)]
        try:
            with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
                yaml.safe_dump(params, config)
                config.flush()
                with self.process(["ros2", "launch", package, "mapping_" + sensor + ".launch.py", "rviz:=false", "use_sim_time:=true", "params_file:=" + config.name]) as proc:
                    self.assertTrue(self.wait_for(lambda: cloud_pub.get_subscription_count() > 0))
                    if package == "pv_lio":
                        self.assertTrue(self.wait_for(lambda: imu_pub.get_subscription_count() > 0))
                    for frame in range(36):
                        self.assertIsNone(proc.poll())
                        base = 1000.0 + frame * .1
                        for k in range(21):
                            sec = base + k * .005
                            ns = round(sec * 1e9)
                            stamp = Time(sec=ns // 1000000000, nanosec=ns % 1000000000)
                            imu = Imu(header=Header(stamp=stamp, frame_id="imu"))
                            imu.linear_acceleration.z = 9.81
                            imu_pub.publish(imu)
                        ns = round(base * 1e9)
                        stamp = Time(sec=ns // 1000000000, nanosec=ns % 1000000000)
                        clock_ns = ns + 100000000
                        clock_pub.publish(Clock(clock=Time(sec=clock_ns // 1000000000, nanosec=clock_ns % 1000000000)))
                        frame_points = [(*p[:-1], base + p[-1]) for p in points] if sensor == "robosenseairy" else points
                        cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id="lidar"), fields, frame_points))
                        until = time.monotonic() + .08
                        while time.monotonic() < until:
                            rclpy.spin_once(self.node, timeout_sec=.01)
                    self.assertTrue(self.wait_for(lambda: len(odometry) >= 3 and clouds and transforms and planes),
                                    f"Missing outputs: odom={len(odometry)}, cloud={len(clouds)}, tf={len(transforms)}, planes={len(planes)}")
                    for msg in odometry:
                        p = msg.pose.pose.position
                        q = msg.pose.pose.orientation
                        self.assertTrue(all(math.isfinite(x) for x in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]))
                        self.assertAlmostEqual(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, places=4)
                        self.assertEqual(msg.header.frame_id, "camera_init")
                        self.assertTrue(1000 <= msg.header.stamp.sec <= 1004)
                    self.assertGreater(clouds[-1].width, 0)
                    self.assertTrue(any(marker.action == marker.ADD and marker.color.a > 0
                                        for message in planes for marker in message.markers))
                    self.assertTrue(self.wait_for(lambda: global_maps))
                    self.assertGreater(global_maps[-1].width, 0)
                    self.assertEqual(global_maps[-1].header.frame_id, "camera_init")
                    late_maps, late_planes = [], []
                    subscriptions.extend([
                        self.node.create_subscription(PointCloud2, "/Laser_map", late_maps.append, snapshot_qos),
                        self.node.create_subscription(MarkerArray, "/planes", late_planes.append, snapshot_qos)])
                    self.assertTrue(self.wait_for(lambda: late_maps and late_planes),
                                    "Late RViz subscribers did not receive retained map snapshots")
                    frames = {(t.header.frame_id, t.child_frame_id) for msg in transforms for t in msg.transforms}
                    self.assertIn(("camera_init", "body" if package == "pv_lio" else "aft_mapped"), frames)
        finally:
            for sub in subscriptions:
                self.node.destroy_subscription(sub)
            for pub in [cloud_pub, imu_pub, clock_pub]:
                self.node.destroy_publisher(pub)

    def test_02_voxelmap_pointcloud_pipeline(self):
        self.pipeline("voxel_map")

    def test_03_pvlio_imu_pointcloud_pipeline(self):
        self.pipeline("pv_lio")

    def test_05_voxelmap_airy_imu_pipeline(self):
        self.pipeline("voxel_map", "robosenseairy")

    def test_04_converters(self):
        for executable, topic, intensity_type in [("rs_to_velodyne", "/rslidar_points", 2),
                                                   ("pandar_to_velodyne", "/hesai/pandar", 7)]:
            with self.subTest(converter=executable):
                outputs = []
                pub = self.node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
                sub = self.node.create_subscription(PointCloud2, "/velodyne_points", outputs.append, 10)
                try:
                    with self.process(["ros2", "run", "pv_lio", executable]):
                        self.assertTrue(self.wait_for(lambda: pub.get_subscription_count() > 0))
                        fields = [PointField(name=n, offset=off, datatype=typ, count=1) for n,off,typ in
                                  [("x",0,7),("y",4,7),("z",8,7),("intensity",12,intensity_type),("ring",16,4),("timestamp",24,8)]]
                        points = [(1.,2.,3.,20 if intensity_type == 2 else 20.,2,1000.0),
                                  (2.,3.,4.,30 if intensity_type == 2 else 30.,3,1000.02)]
                        sample = point_cloud2.create_cloud(Header(stamp=Time(sec=1000),frame_id="lidar"),fields,points)
                        deadline = time.monotonic() + 8
                        while not outputs and time.monotonic() < deadline:
                            pub.publish(sample)
                            rclpy.spin_once(self.node, timeout_sec=.1)
                        self.assertTrue(outputs)
                        result = list(point_cloud2.read_points(outputs[-1],field_names=["x","ring","time"]))
                        self.assertEqual(len(result),2)
                        self.assertEqual(result[1][1],3)
                        self.assertAlmostEqual(result[1][2],.02,places=5)
                        self.assertEqual(outputs[-1].header.stamp.sec,1000)
                finally:
                    self.node.destroy_subscription(sub)
                    self.node.destroy_publisher(pub)


if __name__ == "__main__":
    unittest.main(verbosity=2)
