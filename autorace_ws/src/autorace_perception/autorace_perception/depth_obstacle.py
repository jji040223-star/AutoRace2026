"""뎁스 장애물 노드 (v3 4장 /depth_obstacle, M7 3차선).

  sub  depth_topic   (기본 /camera/depth/image_raw, 16UC1 mm — 실차 Orbbec 드라이버·autorace_sim depth_converter 같은 형식)
  sub  info_topic    (기본 /camera/depth/camera_info) — 내부 파라미터 (fx, fy, cx, cy)
  pub  obstacles_topic (기본 /perception/obstacles, autorace_msgs/ObstacleArray) — 뎁스 영상마다, base_link 기준
  pub  marker_topic  (기본 /perception/obstacle_markers, visualization_msgs/MarkerArray) — RViz 확인용 상자

처리: 뎁스 점 (step 픽셀마다) → 카메라 높이·기울기로 바닥 기준 좌표 → 높이 min_height~max_height, 앞 max_range, 옆 ±max_lateral 안의 점만
→ 바닥 격자 (cell) 에 찍어 붙어 있는 칸끼리 묶음 (OpenCV connectedComponents) → 점 min_points 이상 묶음 = 장애물.
YOLO 학습이 필요 없고 거리가 직접 나온다. 방지턱(2 cm)·노면표시는 min_height 로 걸러진다.
"""
import math
import signal

import cv2
import numpy as np
import rclpy
from autorace_msgs.msg import Obstacle, ObstacleArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray


class DepthObstacle(Node):
    def __init__(self):
        super().__init__('depth_obstacle')
        p = self.declare_parameter
        p('depth_topic', '/camera/depth/image_raw')
        p('info_topic', '/camera/depth/camera_info')
        p('obstacles_topic', '/perception/obstacles')
        p('marker_topic', '/perception/obstacle_markers')
        p('base_frame', 'base_link')
        p('cam_x', 0.084)                  # base_link → 카메라 [m] (sensor_mounts camera_link)
        p('cam_y', 0.0)
        p('cam_height', 0.175)             # 바닥 → 카메라 광심 [m]
        p('cam_pitch_deg', 0.0)            # 아래로 +
        p('cam_yaw_deg', 0.0)              # 카메라가 차 진행 방향보다 왼쪽을 보면 + (lane_calib 으로 잼)
        p('step', 4)                       # 픽셀 건너뛰기 (640×480 → 160×120)
        p('min_height', 0.04)              # [m] 바닥에서 이 높이 이상만 (방지턱 0.02 m 걸러짐)
        p('max_height', 0.6)
        p('max_range', 2.5)                # 앞 [m]
        p('max_lateral', 1.0)              # 옆 ± [m] (3차선 폭 ±0.78 m 보다 조금 넓게)
        p('cell', 0.04)                    # 묶음 격자 [m]
        p('min_points', 8)
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        self.K = None
        self._uv = None
        self.pub = self.create_publisher(ObstacleArray, g('obstacles_topic'), 10)
        self.mk_pub = self.create_publisher(MarkerArray, g('marker_topic'), 2)
        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic'), self.on_depth, qos_profile_sensor_data)
        self.get_logger().info(f"depth_obstacle: {g('depth_topic')} → {g('obstacles_topic')} "
                               f"(높이 {g('min_height')}–{g('max_height')} m, 앞 {g('max_range')} m, 옆 ±{g('max_lateral')} m)")
        self._n = 0
        self._last_log = 0.0

    def on_info(self, msg):
        if self.K is None and msg.k[0] > 0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.get_logger().info(f'카메라 내부 파라미터 fx {msg.k[0]:.1f} fy {msg.k[4]:.1f} cx {msg.k[2]:.1f} cy {msg.k[5]:.1f}')

    def on_depth(self, msg):
        if self.K is None or msg.encoding != '16UC1':
            if msg.encoding != '16UC1':
                self.get_logger().warn(f'16UC1 이 아님: {msg.encoding}', throttle_duration_sec=5.0)
            return
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        st = int(g('step'))
        d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width][::st, ::st]
        if self._uv is None or self._uv[0].shape != d.shape:
            v, u = np.mgrid[0:msg.height:st, 0:msg.width:st]
            self._uv = (u.astype(np.float32), v.astype(np.float32))
        fx, fy, cx, cy = self.K
        u, v = self._uv
        z = d.astype(np.float32) * 0.001            # 광축 방향 [m]
        ok = z > 0
        z, u, v = z[ok], u[ok], v[ok]
        xr = (u - cx) * z / fx                      # 오른쪽
        yd = (v - cy) * z / fy                      # 아래
        th = math.radians(g('cam_pitch_deg'))       # 아래로 숙이면 광축이 바닥 쪽
        fwd = z * math.cos(th) - yd * math.sin(th)
        down = z * math.sin(th) + yd * math.cos(th)
        ps = math.radians(g('cam_yaw_deg'))
        lft = -xr
        x = fwd * math.cos(ps) - lft * math.sin(ps) + g('cam_x')
        y = fwd * math.sin(ps) + lft * math.cos(ps) + g('cam_y')
        h = g('cam_height') - down
        keep = (h >= g('min_height')) & (h <= g('max_height')) & (x <= g('max_range') + g('cam_x')) & (np.abs(y) <= g('max_lateral'))
        x, y, h = x[keep], y[keep], h[keep]

        out = ObstacleArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = g('base_frame')
        if x.size:
            c = float(g('cell'))
            nx = int(math.ceil((g('max_range') + g('cam_x')) / c)) + 1
            ny = int(math.ceil(2 * g('max_lateral') / c)) + 1
            ix = np.clip((x / c).astype(np.int32), 0, nx - 1)
            iy = np.clip(((y + g('max_lateral')) / c).astype(np.int32), 0, ny - 1)
            grid = np.zeros((nx, ny), np.uint8)
            grid[ix, iy] = 255
            n, lab = cv2.connectedComponents(grid, connectivity=8)
            pl = lab[ix, iy]
            for k in range(1, n):
                m = pl == k
                if np.count_nonzero(m) < g('min_points'):
                    continue
                out.obstacles.append(Obstacle(x_near=float(x[m].min()), x_far=float(x[m].max()), y_left=float(y[m].max()),
                                              y_right=float(y[m].min()), height=float(h[m].max()), bottom=float(h[m].min()), points=int(np.count_nonzero(m))))
        out.obstacles.sort(key=lambda o: o.x_near)
        self.pub.publish(out)
        self.publish_markers(out)
        self._n += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_log >= 2.0:
            self._last_log = now
            desc = ', '.join(f'앞 {o.x_near:.2f} m 옆 {o.y_right:+.2f}~{o.y_left:+.2f} 높이 {o.height:.2f}' for o in out.obstacles[:3]) or '없음'
            self.get_logger().info(f'장애물 {len(out.obstacles)} 개: {desc}')

    def publish_markers(self, arr):
        ma = MarkerArray()
        ma.markers.append(Marker(header=arr.header, ns='obstacles', action=Marker.DELETEALL))
        for i, o in enumerate(arr.obstacles):
            mk = Marker(header=arr.header, ns='obstacles', id=i, type=Marker.CUBE, action=Marker.ADD)
            mk.pose.position.x = (o.x_near + o.x_far) / 2
            mk.pose.position.y = (o.y_left + o.y_right) / 2
            mk.pose.position.z = o.height / 2
            mk.pose.orientation.w = 1.0
            mk.scale.x = max(0.02, o.x_far - o.x_near)
            mk.scale.y = max(0.02, o.y_left - o.y_right)
            mk.scale.z = max(0.02, o.height)
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.4, 0.0, 0.6
            ma.markers.append(mk)
        self.mk_pub.publish(ma)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DepthObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
