#!/usr/bin/env python3
"""실차 bag 으로 아커만 조향 확인: 명령 곡률 (cmd_vel ω/v) vs 실제 곡률 (IMU wz / odom v).

  python3 steer_check.py <bag 폴더> [--kmax 2.16]
ROS2 환경 + autorace_msgs (우리 워크스페이스 source) 에서. 출력:
  1) 움직인 시간 중 명령 곡률이 limo_base 한도 (kmax, 안쪽 바퀴 28° → 반경 0.46 m) 를 넘은 비율
  2) 한도 안 회전에서 실제/명령 곡률 비 (중앙값) — 1 보다 작으면 덜 돎 (조향 배율·지연)
  3) 직진 명령 (|k| < 0.1) 때 실제 곡률 평균 — 0 이 아니면 조향 영점 치우침
  4) 1 s 구간 표: 시각, 미션 상태, v, 명령 k, 실제 k, 한도 넘음 표시, 차선 offset
"""
import argparse
import bisect
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    use = [t for t in topics if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=use))
    cls = {t: get_message(types[t]) for t in use}
    out = {t: [] for t in topics}
    while r.has_next():
        t, data, ts = r.read_next()
        out[t].append((ts * 1e-9, deserialize_message(data, cls[t])))
    return out


def near(series, t):
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if not series:
        return None
    if i >= len(series):
        i = len(series) - 1
    return series[i][1]


def med(a):
    a = sorted(a)
    return a[len(a) // 2] if a else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--kmax', type=float, default=2.16)
    ap.add_argument('--win', type=float, default=1.0)
    a = ap.parse_args()
    d = read(a.bag, ['/cmd_vel', '/imu', '/odom', '/mission/state', '/lane/state'])
    cmd, imu, odom = d['/cmd_vel'], d['/imu'], d['/odom']
    print(f'메시지 수: cmd_vel {len(cmd)}, imu {len(imu)}, odom {len(odom)}, mission/state {len(d["/mission/state"])}, '
          f'lane/state {len(d["/lane/state"])}')
    if not cmd or not imu or not odom:
        raise SystemExit('cmd_vel / imu / odom 중 비어 있는 것이 있음')
    t0 = cmd[0][0]
    rows, sat, moving, ratio, straight = [], 0, 0, [], []
    for t, m in cmd:
        v, w = m.linear.x, m.angular.z
        if abs(v) < 0.05:
            continue
        moving += 1
        kc = w / v
        o, i = near(odom, t + 0.25), near(imu, t + 0.25)   # 구동·조향 지연 0.25 s 뒤 실제
        if o is None or i is None or abs(o.twist.twist.linear.x) < 0.05:
            continue
        ka = i.angular_velocity.z / o.twist.twist.linear.x
        if abs(kc) > a.kmax:
            sat += 1
        elif abs(kc) > 0.5:
            ratio.append(ka / kc)
        if abs(kc) < 0.1:
            straight.append(ka)
        rows.append((t - t0, v, kc, ka))
    vr = [abs(near(odom, t + 0.25).twist.twist.linear.x) / abs(m.linear.x) for t, m in cmd if abs(m.linear.x) > 0.1]
    wr = [near(imu, t + 0.25).angular_velocity.z / m.angular.z for t, m in cmd if abs(m.angular.z) > 0.3 and abs(m.linear.x) > 0.1]
    wo = [near(odom, t + 0.25).twist.twist.angular.z / m.angular.z for t, m in cmd if abs(m.angular.z) > 0.3 and abs(m.linear.x) > 0.1]
    print(f'0) 원시값 비 (중앙값): odom v / 명령 v = {med(vr):.2f}, IMU wz / 명령 ω = {med(wr):.2f}, odom wz / 명령 ω = {med(wo):.2f}  '
          f'(명령 v 최대 {max(abs(m.linear.x) for _, m in cmd):.2f}, odom v 최대 {max(abs(o.twist.twist.linear.x) for _, o in odom):.2f}, '
          f'IMU |wz| 최대 {max(abs(i.angular_velocity.z) for _, i in imu):.2f}, 명령 |ω| 최대 {max(abs(m.angular.z) for _, m in cmd):.2f})')
    print(f'\n1) 움직인 명령 {moving} 개 중 곡률 한도 {a.kmax} 넘음: {sat} ({100 * sat / max(1, moving):.1f} %) — 넘은 만큼 덜 돎')
    print(f'2) 한도 안 회전 (0.5 < |k| < {a.kmax}) 실제/명령 비 중앙값: {med(ratio):.2f} (n {len(ratio)}) — 1 이면 정상')
    print(f'3) 직진 명령 때 실제 곡률 평균: {sum(straight) / max(1, len(straight)):+.3f} 1/m (n {len(straight)}) — '
          f'+ 면 왼쪽으로 치우침 (반경 {1 / max(1e-6, abs(sum(straight) / max(1, len(straight)))):.1f} m)')
    st = d['/mission/state']
    ln = d['/lane/state']
    print(f'\n4) {a.win:.0f} s 구간 (명령 k / 실제 k, ! = 한도 넘음)')
    print(f'{"t[s]":>6} {"상태":<12} {"v":>5} {"k명령":>7} {"k실제":>7} {"offset":>7}')
    b = 0.0
    while b < rows[-1][0]:
        seg = [r for r in rows if b <= r[0] < b + a.win]
        if seg:
            kc = max((r[2] for r in seg), key=abs)
            ka = sum(r[3] for r in seg) / len(seg)
            s = near(st, t0 + b)
            l = near(ln, t0 + b)
            off = getattr(l, 'center_offset_m', float('nan')) if l is not None else float('nan')
            print(f'{b:6.0f} {(s.data if s else "-"):<12.12} {sum(r[1] for r in seg) / len(seg):5.2f} '
                  f'{kc:+7.2f}{"!" if abs(kc) > a.kmax else " "} {ka:+7.2f} {off:+7.3f}')
        b += a.win


if __name__ == '__main__':
    main()
