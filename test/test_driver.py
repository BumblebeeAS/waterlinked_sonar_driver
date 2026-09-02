"""Runs the driver against a fake sonar and checks its lifecycle, outputs and services."""

import json
import os
import signal
import sys
import time
import unittest
import urllib.request

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import SetBool
from waterlinked_sonar_driver.srv import SetRange

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
HTTP_PORT = 18080
UDP_PORT = 14747
NODE = "/waterlinked_sonar_driver"
INVALID_NODE = "/invalid_driver"
FRAME_ID = "sonar_link"


@pytest.mark.launch_test
def generate_test_description():
    fake_sonar = launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(TEST_DIR, "fake_sonar.py"),
            os.path.join(TEST_DIR, "data", "ship_short.sonar"),
            str(HTTP_PORT),
        ],
        output="screen",
    )
    connection = {
        "ip_address": "127.0.0.1",
        "http_port": HTTP_PORT,
        "http_timeout": 2.0,
        "udp.mode": "unicast",
        "udp.interface_ip": "127.0.0.1",
        "udp.port": UDP_PORT,
        "udp.unicast_destination_ip": "127.0.0.1",
        "diagnostics_period": 1.0,
    }
    driver = launch_ros.actions.LifecycleNode(
        package="waterlinked_sonar_driver",
        executable="waterlinked_sonar_driver_node",
        name=NODE[1:],
        namespace="",
        output="screen",
        parameters=[connection],
    )
    invalid_driver = launch_ros.actions.LifecycleNode(
        package="waterlinked_sonar_driver",
        executable="waterlinked_sonar_driver_node",
        name=INVALID_NODE[1:],
        namespace="",
        output="screen",
        parameters=[{**connection, "range_min": 5.0, "range_max": 1.0}],
    )
    return launch.LaunchDescription(
        [fake_sonar, driver, invalid_driver, launch_testing.actions.ReadyToTest()]
    ), {"driver": driver}


def sonar_get(path, timeout_sec=10.0):
    url = f"http://127.0.0.1:{HTTP_PORT}/api/v1/integration{path}"
    deadline = time.monotonic() + timeout_sec
    while True:
        try:
            with urllib.request.urlopen(url, timeout=1.0) as response:
                return json.load(response)
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.1)


class TestDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_driver")
        sonar_get("/about")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.node.destroy_client(client)
        self.assertIsNotNone(future.result(), name)
        return future.result()

    def transition(self, node, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self.call(ChangeState, node + "/change_state", request).success

    def state(self, node):
        return self.call(
            GetState, node + "/get_state", GetState.Request()
        ).current_state.id

    def receive(self, topics, count, timeout_sec=30.0):
        """Returns at least count messages from each of the (type, topic) pairs."""
        received = {topic: [] for _, topic in topics}
        subscriptions = [
            self.node.create_subscription(
                message_type, topic, received[topic].append, 10
            )
            for message_type, topic in topics
        ]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and min(map(len, received.values())) < count:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
        for topic, messages in received.items():
            self.assertGreaterEqual(len(messages), count, topic)
        return received

    def test_invalid_parameters_fail_configuration(self):
        self.assertFalse(self.transition(INVALID_NODE, Transition.TRANSITION_CONFIGURE))
        self.assertEqual(self.state(INVALID_NODE), State.PRIMARY_STATE_UNCONFIGURED)

    def test_lifecycle_outputs_and_services(self, driver, proc_info):
        self.assertTrue(self.transition(NODE, Transition.TRANSITION_CONFIGURE))
        self.assertEqual(self.state(NODE), State.PRIMARY_STATE_INACTIVE)
        self.assertTrue(self.transition(NODE, Transition.TRANSITION_ACTIVATE))
        self.assertEqual(self.state(NODE), State.PRIMARY_STATE_ACTIVE)
        self.assertTrue(sonar_get("/acoustics/enabled"))

        received = self.receive(
            [
                (PointCloud2, NODE + "/point_cloud"),
                (Image, NODE + "/range_image"),
                (Image, NODE + "/intensity_image"),
                (CameraInfo, NODE + "/camera_info"),
            ],
            count=3,
        )
        self.check_range_images(received[NODE + "/range_image"])
        self.check_intensity_images(received[NODE + "/intensity_image"])
        self.check_camera_info(received[NODE + "/camera_info"])
        self.check_clouds_match_range_images(
            received[NODE + "/point_cloud"], received[NODE + "/range_image"]
        )

        self.check_services()

        self.assertTrue(self.transition(NODE, Transition.TRANSITION_DEACTIVATE))
        self.assertEqual(self.state(NODE), State.PRIMARY_STATE_INACTIVE)
        self.assertFalse(sonar_get("/acoustics/enabled"))

        request = SetBool.Request(data=True)
        self.assertTrue(self.call(SetBool, NODE + "/enable_acoustics", request).success)
        self.assertTrue(sonar_get("/acoustics/enabled"))
        os.kill(driver.process_details["pid"], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=driver, timeout=10)
        self.assertFalse(sonar_get("/acoustics/enabled"))

    def check_range_images(self, images):
        for image in images:
            self.assertEqual(image.header.frame_id, FRAME_ID)
            self.assertEqual(image.encoding, "32FC1")
            self.assertEqual((image.width, image.height), (256, 64))
            distances = np.frombuffer(bytes(image.data), np.float32)
            self.assertEqual(distances.size, image.width * image.height)
            self.assertTrue(np.any(distances > 0.0))

    def check_intensity_images(self, images):
        for image in images:
            self.assertEqual(image.header.frame_id, FRAME_ID)
            self.assertEqual(image.encoding, "8UC1")
            self.assertEqual(len(image.data), image.width * image.height)

    def check_camera_info(self, infos):
        for info in infos:
            self.assertEqual(info.header.frame_id, FRAME_ID)
            self.assertGreater(info.k[0], 0.0)
            self.assertGreater(info.k[4], 0.0)
            self.assertEqual(info.k[2], info.width / 2.0)
            self.assertEqual(info.k[5], info.height / 2.0)

    def check_clouds_match_range_images(self, clouds, images):
        """Each point lies at the distance of its pixel in the range image of the same shot."""
        images_by_stamp = {
            (i.header.stamp.sec, i.header.stamp.nanosec): i for i in images
        }
        matched = 0
        for cloud in clouds:
            self.assertEqual(cloud.header.frame_id, FRAME_ID)
            self.assertEqual([f.name for f in cloud.fields], ["x", "y", "z"])
            image = images_by_stamp.get(
                (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
            )
            if image is None:
                continue
            points = np.frombuffer(bytes(cloud.data), np.float32).reshape(-1, 3)
            distances = np.frombuffer(bytes(image.data), np.float32)
            valid = distances[distances > 0.0]
            self.assertEqual(len(points), len(valid))
            np.testing.assert_allclose(np.linalg.norm(points, axis=1), valid, rtol=1e-5)
            matched += 1
        self.assertGreater(matched, 0)

    def check_services(self):
        request = SetRange.Request(min=1.0, max=10.0)
        self.assertTrue(self.call(SetRange, NODE + "/set_range", request).success)
        self.assertEqual(sonar_get("/acoustics/range"), {"min": 1.0, "max": 10.0})

        request = SetRange.Request(min=5.0, max=2.0)
        self.assertFalse(self.call(SetRange, NODE + "/set_range", request).success)
        self.assertEqual(sonar_get("/acoustics/range"), {"min": 1.0, "max": 10.0})

        request = SetBool.Request(data=True)
        self.assertTrue(
            self.call(SetBool, NODE + "/enable_high_frequency", request).success
        )
        self.assertEqual(sonar_get("/acoustics/mode"), "high-frequency")


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
