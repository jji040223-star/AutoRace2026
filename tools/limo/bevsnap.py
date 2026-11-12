# 지금 카메라 한 장 + lane_follower 디버그 (BEV) 한 장을 PNG 로. 사용: python3 bevsnap.py <폴더>
import sys, os, time, numpy as np, cv2, rclpy
from sensor_msgs.msg import Image
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
rclpy.init(); n = rclpy.create_node("bevsnap"); got = {}
def cb(name):
    def f(m):
        if name in got: return
        ch = 4 if m.encoding.endswith("a8") else 3
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // ch, ch)[:, :m.width, :3]
        if m.encoding.startswith("rgb"): img = img[:, :, ::-1]
        got[name] = img.copy()
    return f
n.create_subscription(Image, "/camera/color/image_raw", cb("raw"), 1)
n.create_subscription(Image, "/lane/debug", cb("debug"), 1)
t = time.time()
while len(got) < 2 and time.time() - t < 15: rclpy.spin_once(n, timeout_sec=0.2)
for k, v in got.items(): cv2.imwrite(f"{out}/{k}.png", v); print(k, v.shape)
