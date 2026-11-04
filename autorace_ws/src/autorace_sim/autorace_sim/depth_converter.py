"""Isaac 뎁스(32FC1, m) → 실차 Orbbec 드라이버와 같은 16UC1(mm). 유효 거리 밖·NaN·inf 는 0."""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthConverter(Node):
    def __init__(self):
        super().__init__("depth_converter")
        self.declare_parameter("input_topic", "/isaac/camera/depth/image_raw")
        self.declare_parameter("output_topic", "/camera/depth/image_raw")
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("max_depth", 3.0)
        self.lo = self.get_parameter("min_depth").value
        self.hi = self.get_parameter("max_depth").value
        self.pub = self.create_publisher(Image, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("input_topic").value, self.on_depth, qos_profile_sensor_data)

    def on_depth(self, msg):
        if msg.encoding != "32FC1":
            self.get_logger().error(f"32FC1 이 아님: {msg.encoding}", throttle_duration_sec=5.0)
            return
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        valid = np.isfinite(d) & (d >= self.lo) & (d <= self.hi)
        mm = np.where(valid, np.round(d * 1000.0), 0).astype(np.uint16)
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding="16UC1",
                    is_bigendian=0, step=msg.width * 2, data=mm.tobytes())
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(DepthConverter())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
