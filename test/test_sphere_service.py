"""ROS integration test; run in an isolated ROS_DOMAIN_ID after sourcing overlay.

Starts its own filter process and uses synthetic metre-depth frames and static TF.
No simulator, robot controllers, or mapper are contacted.
"""
import array
import math
import subprocess
import tempfile
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from sphere_scene_interfaces.msg import Sphere
from sphere_scene_interfaces.srv import SetSphereSet
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster


def main():
    rclpy.init()
    node = Node("sphere_service_test")
    temporary = tempfile.TemporaryDirectory()
    urdf = Path(temporary.name) / "robot.urdf"
    geometry = '<collision><geometry><box size="1 1 1"/></geometry></collision>'
    urdf.write_text('<robot name="test"><link name="test_robot">' + geometry +
                    '</link><link name="test_robot_missing">' + geometry +
                    '</link><joint name="fixed" type="fixed"><parent link="test_robot"/>'
                    '<child link="test_robot_missing"/></joint></robot>')
    process = subprocess.Popen([
        get_package_prefix("depth_self_filter") + "/lib/depth_self_filter/depth_self_filter_node",
        "--ros-args", "-p", "filtering_enabled:=false", "-p", "sphere_padding_m:=0.0",
        "-p", "tf_timeout_s:=0.0",
        "-p", "require_complete_robot_tf:=true", "-p", f"robot_description_file:={urdf}",
    ])
    try:
        client = node.create_client(SetSphereSet, "/depth_self_filter/set_sphere_set")
        gate = node.create_client(SetBool, "/depth_self_filter/set_enabled")
        assert client.wait_for_service(timeout_sec=15)
        assert gate.wait_for_service(timeout_sec=15)
        pub = node.create_publisher(Image, "image", qos_profile_sensor_data)
        cam_pub = node.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
        received = []
        node.create_subscription(Image, "image_filtered", received.append, qos_profile_sensor_data)
        broadcaster = StaticTransformBroadcaster(node)

        def tf(child, x=0.):
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = "test_camera"
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transform.transform.translation.x = x
            broadcaster.sendTransform(transform)

        def call(service, request):
            future = service.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), "service timed out"
            return future.result()

        def frames(duration=0.8):
            received.clear()
            cutoff = node.get_clock().now().nanoseconds
            end = time.monotonic() + duration
            while time.monotonic() < end:
                image = Image()
                image.header.stamp = node.get_clock().now().to_msg()
                image.header.frame_id = "test_camera"
                image.height, image.width, image.step = 3, 5, 20
                image.encoding = "32FC1"
                values = [2.0] * 15
                values[5:10] = [1.0, 1.8, 2.0, 2.2, 3.0]
                image.data = array.array("f", values).tobytes()
                info = CameraInfo()
                info.header = image.header
                info.height, info.width = 3, 5
                info.k = [10.0, 0.0, 2.0, 0.0, 10.0, 1.0, 0.0, 0.0, 1.0]
                pub.publish(image)
                cam_pub.publish(info)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.02)
            return [m for m in received if
                    m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec >= cutoff]

        tf("tracked")
        tf("test_robot", x=10.)
        assert not frames(), "disabled filter leaked frames before registration"
        request = SetSphereSet.Request()
        request.set.id = "test"
        request.set.revision = 1
        request.set.parent_frame = "tracked"
        request.set.pose.orientation.w = 1.0
        sphere = Sphere()
        sphere.center.z, sphere.radius = 2.0, 0.3
        request.set.spheres = [sphere]
        assert call(client, request).success
        assert call(client, request).success, "retry was not idempotent"
        enable = SetBool.Request()
        enable.data = True
        assert call(gate, enable).success
        assert not frames(), "one missing robot TF leaked a partially filtered frame"
        tf("test_robot_missing", x=11.)
        output = frames(1.5)
        assert output, "no filtered frames"
        values = array.array("f", bytes(output[-1].data))
        assert values[5] == 1.0 and values[9] == 3.0
        assert all(math.isnan(values[i]) for i in [6, 7, 8])
        request.set.revision = 2
        request.set.spheres[0].radius = float("nan")
        assert not call(client, request).success
        request.set.spheres[0].radius = 0.3
        request.set.parent_frame = "missing_tf"
        assert call(client, request).success
        assert not frames(), "missing object TF leaked unfiltered frames"
        tf("missing_tf")
        assert frames(1.5), "filter did not recover when TF became available"
        request.set.revision = 3
        request.remove = True
        assert call(client, request).success
        assert call(client, request).success
        output = frames()
        assert output and all(math.isfinite(x) for x in array.array("f", bytes(output[-1].data)))
        request.remove = False
        request.set.revision = 2
        assert not call(client, request).success, "stale request resurrected removed geometry"
        print("PASS: registration, retries, validation, timestamped filtering, complete robot/object TF, removal")
    finally:
        process.terminate()
        process.wait(timeout=10)
        temporary.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
