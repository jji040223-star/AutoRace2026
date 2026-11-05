"""YOLO 표지판 검출 노드 (실차·시뮬 공통).

  sub  image_topic       (기본 /camera/color/image_raw, sensor_msgs/Image rgb8|bgr8)
  pub  detections_topic  (기본 /perception/detections, autorace_msgs/DetectionArray) — 처리한 영상마다 하나, 없으면 빈 배열
  pub  debug_topic       (기본 /perception/debug, sensor_msgs/Image bgr8, debug_hz 로 제한) — 박스·라벨을 그린 영상

모델: github.com/mark2455/2026_Autorace yolo-training/models/best.pt (YOLOv8n, 640, 6 클래스
crosswalk·parking·slow·speed_bump·turn_a·turn_b) 를 models/autorace_yolov8n.pt 로 복사해 둠.
영상은 콜백에서 최신 한 장만 들고 있고 타이머(max_hz)로 처리한다 — 추론이 느려도 밀린 영상을 쌓지 않는다.

M2 갈림길 표지판 보조 (branch_assist, branch_sign.py): 이 모델은 시뮬 렌더 A/B 표지판을 1 m 앞에서도 못 잡고,
가까운 정차 LIMO 바퀴를 turn_a 로 잡았다 (2026-11-01). 그래서
  ① YOLO turn_a/turn_b 박스 안에 파란 원이 없으면 버린다 (branch_check_yolo)
  ② YOLO 가 안 잡은 파란 원은 A/B 그림과 상관으로 판별해 같은 라벨로 넣는다 (상관 ≥ branch_min_score, 반대쪽보다 branch_min_margin 이상)
"""
import signal
import time

import cv2
import numpy as np
import rclpy
from autorace_msgs.msg import Detection, DetectionArray
from autorace_perception.branch_sign import BranchSign
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        p = self.declare_parameter
        p('model_path', '')                # 비우면 이 패키지 models/autorace_yolov8n.pt
        p('image_topic', '/camera/color/image_raw')
        p('detections_topic', '/perception/detections')
        p('debug_topic', '/perception/debug')
        p('debug_hz', 5.0)                 # 0 = 디버그 영상 안 냄
        p('conf', 0.5)                     # 이 미만 검출은 버림
        p('imgsz', 640)                    # 학습 크기 (args.yaml imgsz 640)
        p('device', '')                    # '' = CUDA 있으면 0, 없으면 cpu
        p('max_hz', 10.0)                  # 추론 상한 (실차 젯슨은 낮춘다)
        p('branch_assist', True)           # M2 갈림길 표지판 보조 (위 설명)
        p('branch_labels', ['turn_a', 'turn_b'])
        p('branch_templates', [''])        # 라벨 순서대로 그림 경로. 비우면 이 패키지 models/sign_branch_A.png, sign_branch_B.png
        p('branch_min_px', 12)             # 파란 원 최소 지름 [px]
        p('branch_min_score', 0.6)
        p('branch_min_margin', 0.25)
        p('branch_check_yolo', True)       # YOLO turn 박스 안에 파란 원이 있어야 인정
        g = lambda k: self.get_parameter(k).value   # noqa: E731

        from ultralytics import YOLO       # 무거워서 노드 생성 때 import
        import torch
        path = g('model_path')
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = get_package_share_directory('autorace_perception') + '/models/autorace_yolov8n.pt'
        self.device = g('device') or (0 if torch.cuda.is_available() else 'cpu')
        self.model = YOLO(path)
        self.model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=g('imgsz'), device=self.device, verbose=False)  # 첫 추론 지연 미리
        self.names = self.model.names
        self.branch = None
        if g('branch_assist'):
            tpl = [t for t in g('branch_templates') if t]
            if not tpl:
                from ament_index_python.packages import get_package_share_directory
                d = get_package_share_directory('autorace_perception') + '/models/'
                tpl = [d + 'sign_branch_A.png', d + 'sign_branch_B.png']
            self.branch = BranchSign(dict(zip(g('branch_labels'), tpl)), min_px=int(g('branch_min_px')))

        self.det_pub = self.create_publisher(DetectionArray, g('detections_topic'), 10)
        self.dbg_pub = self.create_publisher(Image, g('debug_topic'), 2) if g('debug_hz') > 0 else None
        self.create_subscription(Image, g('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / float(g('max_hz')), self.step)
        self._img = None                   # (msg, bgr) 최신 한 장
        self._last_dbg = 0.0
        self._stat = {'n': 0, 'labels': {}, 't': time.monotonic(), 'ms': 0.0}
        self.get_logger().info(f"yolo_detector: {path} (device {self.device}, classes {list(self.names.values())}), "
                               f"{g('image_topic')} → {g('detections_topic')}, 갈림길 보조 "
                               f"{'켬' if self.branch else '끔'}")

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            self.get_logger().warn(f'지원하지 않는 영상 형식 {msg.encoding}', throttle_duration_sec=5.0)
            return
        self._img = msg

    def step(self):
        msg, self._img = self._img, None
        if msg is None:
            return
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        ch = 4 if msg.encoding.endswith('a8') else 3
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // ch, ch)[:, :msg.width, :3]
        bgr = np.ascontiguousarray(img[:, :, ::-1] if msg.encoding.startswith('rgb') else img)

        t0 = time.monotonic()
        r = self.model.predict(bgr, imgsz=g('imgsz'), conf=g('conf'), device=self.device, verbose=False)[0]
        out = DetectionArray(header=msg.header, image_width=msg.width, image_height=msg.height)
        for b in r.boxes:
            x1, y1, x2, y2 = (int(round(v)) for v in b.xyxy[0].tolist())
            out.detections.append(Detection(label=self.names[int(b.cls)], confidence=float(b.conf),
                                            x1=x1, y1=y1, x2=x2, y2=y2))
        assist = self.branch_assist(bgr, out) if self.branch is not None else []
        self.det_pub.publish(out)

        now = time.monotonic()
        s = self._stat
        s['n'] += 1
        s['ms'] += (now - t0) * 1000.0
        for d in out.detections:
            k = d.label + ('(보조)' if d in assist else '')
            s['labels'][k] = s['labels'].get(k, 0) + 1
        if self.dbg_pub is not None and now - self._last_dbg >= 1.0 / float(g('debug_hz')):
            self._last_dbg = now
            dbg = bgr.copy()
            for d in out.detections:
                col = (255, 128, 0) if d in assist else (0, 255, 0)     # 갈림길 보조 = 파랑
                cv2.rectangle(dbg, (d.x1, d.y1), (d.x2, d.y2), col, 2)
                cv2.putText(dbg, f'{d.label} {d.confidence:.2f} h{d.y2 - d.y1}', (d.x1, max(12, d.y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            self.dbg_pub.publish(Image(header=msg.header, height=dbg.shape[0], width=dbg.shape[1], encoding='bgr8',
                                       step=dbg.shape[1] * 3, data=dbg.tobytes()))
        if now - s['t'] >= 2.0:
            seen = ', '.join(f'{k} {v}' for k, v in sorted(s['labels'].items())) or '없음'
            self.get_logger().info(f"{s['n'] / (now - s['t']):.1f} Hz, 추론 {s['ms'] / max(1, s['n']):.0f} ms, 검출: {seen}")
            self._stat = {'n': 0, 'labels': {}, 't': now, 'ms': 0.0}

    def branch_assist(self, bgr, out):
        """YOLO 결과 out 에 갈림길 표지판 보조를 합친다 (out.detections 를 바꿈). 보조로 넣은 검출 목록을 돌려준다."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        labels = set(g('branch_labels'))
        circles = self.branch.circles(bgr)

        def inside(c, d):
            cx, cy = (c[0] + c[2]) / 2, (c[1] + c[3]) / 2
            return d.x1 <= cx <= d.x2 and d.y1 <= cy <= d.y2

        turns = [d for d in out.detections if d.label in labels]
        if g('branch_check_yolo'):
            drop = [d for d in turns if not any(inside(c, d) for c in circles)]
            out.detections = [d for d in out.detections if d not in drop]
            turns = [d for d in turns if d not in drop]
        added = []
        for c in circles:
            if any(inside(c, d) for d in turns):
                continue                    # YOLO 가 이미 잡은 표지판
            sc = sorted(self.branch.classify(bgr, c).items(), key=lambda kv: -kv[1])
            if sc[0][1] >= g('branch_min_score') and sc[0][1] - sc[1][1] >= g('branch_min_margin'):
                d = Detection(label=sc[0][0], confidence=float(sc[0][1]), x1=c[0], y1=c[1], x2=c[2], y2=c[3])
                out.detections.append(d)
                added.append(d)
        return added


def main():
    # lane_follower 와 같게: rclpy 기본 SIGINT 처리기를 끄고, 정리 중에 오는 두 번째 Ctrl+C 는 무시
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
