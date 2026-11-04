"""가짜 신호등 검출 (시뮬 전용, M1 YOLO 모델이 생기기 전) — 시뮬이 아는 신호등 몸체 앞면을 카메라에 투영한 박스를 낸다.

  sub  info_topic        (기본 /camera/color/camera_info), image_topic (박스 시각을 영상에 맞춤), /odom
  pub  detections_topic  (기본 /perception/traffic_light_detections, autorace_msgs/DetectionArray, label traffic_light)

색은 넣지 않는다 — traffic_light 노드가 실제 영상의 세 칸 밝기로 색을 정한다 (YOLO 1 클래스와 같게).
decoy 가 true 면 방해 모니터 화면 위 그래픽 자리 (가운데 그린 신호등 0.094 × 0.16 m, 원 0.12 × 0.12 m) 에도 같은 라벨 박스를 낸다
— YOLO 가 모니터를 잘못 잡는 경우를 흉내 낸다 (모니터에 무엇이 떠 있든).
좌표: world = 출발 자세 (start_x, start_y, start_yaw_deg) 에 odom 을 더한 것. 위치는 LimoIsaacSIM/autorace_2026 config/missions.yaml traffic_light·ad_monitor.
"""
import math
import signal

import rclpy
from autorace_msgs.msg import Detection, DetectionArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image


class FakeTrafficLight(Node):
    def __init__(self):
        super().__init__('fake_traffic_light')
        p = self.declare_parameter
        p('info_topic', '/camera/color/camera_info')
        p('image_topic', '/camera/color/image_raw')
        p('detections_topic', '/perception/traffic_light_detections')
        p('label', 'traffic_light')
        p('start_x', 0.74)
        p('start_y', 5.58)
        p('start_yaw_deg', 0.0)            # 시뮬 출발 자세 (params_sim robot.start_*) — odom 원점
        p('cam_x', 0.084)
        p('cam_z', 0.175)
        # 신호등 몸체 앞면 (world, 차 쪽을 보는 면): x, y 가운데, 폭, 아래·위 z
        p('light_face', [3.95, 4.47, 0.12, 0.10, 0.305])
        p('decoy', False)
        p('decoy_faces', [3.95, 4.20, 0.094, 0.12, 0.28,      # 모니터 가운데 그린 신호등
                          3.95, 4.20, 0.12, 0.14, 0.26])      # 모니터 가운데 원
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.K = None
        self.size = None
        self.pose = None
        self.pub = self.create_publisher(DetectionArray, g('detections_topic'), 10)
        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Image, g('image_topic'), self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"fake_traffic_light: 신호등 {g('light_face')}, 방해 모니터 박스 {'켬' if g('decoy') else '끔'} "
                               f"→ {g('detections_topic')}")

    def on_info(self, msg):
        if msg.k[0] > 0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.size = (msg.width, msg.height)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        y0 = math.radians(g('start_yaw_deg'))
        self.pose = (g('start_x') + p.x * math.cos(y0) - p.y * math.sin(y0), g('start_y') + p.x * math.sin(y0) + p.y * math.cos(y0),
                     yaw + y0)

    def project(self, face):
        """앞면 [x, y, 폭, z0, z1] → 영상 박스 (화면 밖이면 None). 앞면은 진행 방향 (+x 쪽에서 보는) 과 수직이라 가정."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        fx, fy, cx, cy = self.K
        x, y, yaw = self.pose
        camx, camy = x + g('cam_x') * math.cos(yaw), y + g('cam_x') * math.sin(yaw)
        fxw, fyw, w, z0, z1 = face
        us, vs = [], []
        for py in (fyw - w / 2, fyw + w / 2):
            for pz in (z0, z1):
                dx, dy = fxw - camx, py - camy
                f = dx * math.cos(yaw) + dy * math.sin(yaw)
                left = -dx * math.sin(yaw) + dy * math.cos(yaw)
                if f < 0.1:
                    return None
                us.append(cx - fx * left / f)
                vs.append(cy - fy * (pz - g('cam_z')) / f)
        W, H = self.size
        x1, x2, y1, y2 = min(us), max(us), min(vs), max(vs)
        if x2 < 0 or x1 > W or y2 < 0 or y1 > H:
            return None
        # 화면 끝에 반 넘게 잘리면 YOLO 도 못 잡는다고 본다
        vis = (min(x2, W) - max(x1, 0)) * (min(y2, H) - max(y1, 0)) / max(1.0, (x2 - x1) * (y2 - y1))
        if vis < 0.5:
            return None
        return int(max(0, x1)), int(max(0, y1)), int(min(W - 1, x2)), int(min(H - 1, y2))

    def on_image(self, msg):
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        if self.K is None or self.pose is None:
            return
        out = DetectionArray(header=msg.header, image_width=msg.width, image_height=msg.height)
        faces = [(list(g('light_face')), 0.9)]
        if g('decoy'):                     # 신뢰도 0.8 = 모니터 박스 (시험 판정에서 구분용. traffic_light 노드는 신뢰도를 안 씀)
            df = list(g('decoy_faces'))
            faces += [(df[i:i + 5], 0.8) for i in range(0, len(df) - 4, 5)]
        for face, conf in faces:
            b = self.project(face)
            if b is not None:
                out.detections.append(Detection(label=g('label'), confidence=conf, x1=b[0], y1=b[1], x2=b[2], y2=b[3]))
        self.pub.publish(out)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FakeTrafficLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
