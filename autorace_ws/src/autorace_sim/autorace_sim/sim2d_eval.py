"""2D 시뮬 주행 판정 — 미션 상태 변화·주차 결과·선 밟음·충돌을 모아 끝에 요약한다 (시뮬 시간 기준).

  ros2 run autorace_sim sim2d_eval [--until parking_done|mission_done|none] [--timeout 400] [--frames /tmp/f --every 5]

  --until parking_done : /parking/state DONE 또는 FAIL 이면 끝 (기본)
  --frames DIR         : 상태가 바뀔 때와 --every 초마다 카메라 영상·/lane/debug 를 png 로 저장,
                         끝에 경로 (path.csv: 시각, x, y, yaw, 미션 상태) 와 트랙 위 경로 그림 (path.png)
끝나면 마지막 줄에 'SIM2D_RESULT {json}' 을 찍고, 실패 (충돌·라바콘·FAIL·시간 초과) 가 있으면 exit 1.
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String


def to_bgr(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()


def save_path(out, path, events):
    """경로 CSV + 트랙 위 그림 (미션 상태별 색, 충돌·라바콘·선 밟음 표시)."""
    import csv
    from ament_index_python.packages import get_package_share_directory
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'path.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'yaw', 'mission', 'parking'])
        w.writerows(path)
    track = json.load(open(os.path.join(get_package_share_directory('autorace_track'), 'data', 'track_elements.json')))
    sc, mg = 100.0, 20                      # 1 cm/px
    img = np.full((int(6 * sc) + 2 * mg, int(10 * sc) + 2 * mg, 3), (46, 52, 54), np.uint8)

    def px(x, y):
        return int(mg + x * sc), int(mg + (6 - y) * sc)
    for e in track['elements']:
        if not e.get('painted'):
            continue
        col = (67, 170, 229) if e['class'] in ('road_edge', 'zigzag') else (202, 218, 223)
        cv2.fillPoly(img, [np.array([px(x / 1000, y / 1000) for x, y in e['exterior']], np.int32)], col)
    colors = {'LANE': (0, 255, 0), 'M7_AVOID': (255, 0, 255), 'M8_AVOID': (255, 128, 0), 'M4_PARKING': (0, 200, 255)}
    for (t0, x0, y0, *_), (t1, x1, y1, _, mis, _) in zip(path, path[1:]):
        cv2.line(img, px(x0, y0), px(x1, y1), colors.get(mis, (0, 0, 255)), 2)
    for e in events:
        if e['kind'] in ('collide', 'cone') and e['x'] is not None:
            cv2.circle(img, px(e['x'], e['y']), 8, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(out, 'path.png'), img)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--until', default='parking_done', choices=['parking_done', 'mission_done', 'none'])
    ap.add_argument('--timeout', type=float, default=400.0, help='시뮬 시간 [s]')
    ap.add_argument('--frames', default='')
    ap.add_argument('--every', type=float, default=0.0)
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = rclpy.create_node('sim2d_eval')
    st = {'status': None, 'mission': None, 'parking': None, 'events': [], 'parking_result': '', 't0': None,
          'img': None, 'dbg': None, 'last_save': -1e9, 'saved': 0, 'states': {}}

    def t_now():
        s = st['status']
        return s['t'] if s else 0.0

    def event(kind, val):
        s = st['status'] or {}
        e = dict(t=round(t_now() - (st['t0'] or 0.0), 2), kind=kind, val=val, x=s.get('x'), y=s.get('y'))
        st['events'].append(e)
        print(f"[eval] {e['t']:7.2f}s {kind:8s} → {val:22s} ({e['x']}, {e['y']})", flush=True)
        save(f'{kind}_{val}')

    def save(tag):
        if not a.frames:
            return
        os.makedirs(a.frames, exist_ok=True)
        t = t_now() - (st['t0'] or 0.0)
        for name, img in (('cam', st['img']), ('dbg', st['dbg'])):
            if img is not None:
                cv2.imwrite(os.path.join(a.frames, f'{t:07.2f}_{tag}_{name}.png'), img)
        st['saved'] += 1

    def on_status(m):
        s = json.loads(m.data)
        if st['t0'] is None or (st['status'] and s['t'] < st['status']['t']):
            st['t0'] = s['t']
        prev = st['status']
        st['status'] = s
        if prev is not None:
            for c in set(s['collisions']) - set(prev['collisions']):
                event('collide', c)
            for c in set(s['knocked']) - set(prev['knocked']):
                event('cone', c)
        if a.every > 0 and t_now() - st['last_save'] >= a.every:
            st['last_save'] = t_now()
            save('every')

    def on_state(key):
        def cb(m):
            if m.data != st[key]:
                st[key] = m.data
                st['states'].setdefault(key, []).append(m.data)
                event(key, m.data)
        return cb

    node.create_subscription(String, '/sim2d/status', on_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/mission/state', on_state('mission'), 10)
    node.create_subscription(String, '/parking/state', on_state('parking'), 10)
    node.create_subscription(String, '/parking/result', lambda m: st.update(parking_result=m.data), 10)
    path = []

    def on_pose(m):
        if st['status'] is not None and st['t0'] is not None:
            if not path or math.dist(path[-1][1:3], (m.x, m.y)) > 0.005 or abs(math.remainder(m.theta - path[-1][3], math.tau)) > math.radians(1.0):
                path.append((round(t_now() - st['t0'], 2), m.x, m.y, m.theta, st['mission'] or '', st['parking'] or ''))
    node.create_subscription(Pose2D, '/sim2d/pose', on_pose, 10)
    node.create_subscription(Image, '/camera/color/image_raw', lambda m: st.update(img=to_bgr(m)), qos_profile_sensor_data)
    node.create_subscription(Image, '/lane/debug', lambda m: st.update(dbg=to_bgr(m)), qos_profile_sensor_data)
    reason = 'timeout'
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if st['status'] is None or st['t0'] is None:
                continue
            if t_now() - st['t0'] > a.timeout:
                break
            if a.until == 'parking_done' and st['parking'] in ('DONE', 'FAIL'):
                reason = f"parking {st['parking']}"
                break
            if a.until == 'mission_done' and st['mission'] == 'DONE':
                reason = 'mission DONE'
                break
    except KeyboardInterrupt:
        reason = 'interrupted'
    s = st['status'] or {}
    res = dict(reason=reason, sim_s=round(t_now() - (st['t0'] or 0.0), 1), dist_m=s.get('dist_m'),
               min_line_m=s.get('min_line_m'), touch_s=s.get('touch_s'), collisions=s.get('collisions'), knocked=s.get('knocked'),
               lane_out=s.get('lane_out'), lane_out_s=s.get('lane_out_s'), lane_outs=s.get('lane_outs', []),
               mission_states=st['states'].get('mission', []), parking_states=st['states'].get('parking', []),
               parking_result=st['parking_result'], branch=s.get('branch'), final=dict(x=s.get('x'), y=s.get('y')),
               odom_err_m=s.get('odom_err_m'), frames_saved=st['saved'], cells=s.get('cells', {}), stops=s.get('stops', []))
    if a.frames and path:
        save_path(a.frames, path, st['events'])
    print('SIM2D_RESULT ' + json.dumps(res, ensure_ascii=False), flush=True)
    bad = bool(s.get('collisions')) or bool(s.get('knocked')) or reason in ('timeout', 'parking FAIL')
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if bad else 0)
