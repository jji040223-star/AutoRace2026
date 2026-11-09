#!/usr/bin/env python3
"""rosbag2 (sqlite3) 의 CompressedImage 토픽을 mp4 로.

  python3 bag_to_mp4.py <bag 폴더> [출력.mp4] [--topic /camera/color/image_raw/compressed]
ROS2 환경 (source /opt/ros/humble/setup.bash) 에서 실행. fps 는 메시지 시각으로 계산하고,
프레임이 빠진 구간은 앞 프레임을 반복해 실제 시간과 영상 길이를 맞춘다. 왼쪽 위에 시작 뒤 초를 찍는다.
"""
import argparse
import os

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out', nargs='?')
    ap.add_argument('--topic', default='/camera/color/image_raw/compressed')
    ap.add_argument('--fps', type=float, default=15.0)
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.bag.rstrip('/'))), 'camera.mp4')

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))
    vw, t0, n_in, n_out, last = None, None, 0, 0, None
    while r.has_next():
        _, data, t = r.read_next()
        img = cv2.imdecode(np.frombuffer(deserialize_message(data, CompressedImage).data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        n_in += 1
        if vw is None:
            t0 = t
            h, w = img.shape[:2]
            vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (w, h))
        sec = (t - t0) * 1e-9
        cv2.putText(img, f'{sec:6.1f} s', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        # 실제 시간에 맞춰 프레임 수 채우기
        target = int(sec * a.fps) + 1
        while n_out < target:
            vw.write(img if n_out == target - 1 or last is None else last)
            n_out += 1
        last = img
    if vw is None:
        raise SystemExit(f'{a.topic} 메시지가 없음')
    vw.release()
    print(f'{out}: 입력 {n_in} 장, 출력 {n_out} 프레임 ({n_out / a.fps:.1f} s, {a.fps} fps)')


if __name__ == '__main__':
    main()
