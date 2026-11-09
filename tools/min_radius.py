#!/usr/bin/env python3
"""실차 bag 에서 최대 조향 (명령 곡률이 limo_base 한도를 넘어 최대로 꺾인) 구간의 실제 회전 반경.

  python3 min_radius.py <bag 폴더> [--kmax 2.16] [--min-s 1.0] [--settle-s 0.6]
명령 |ω/v| > kmax 가 min-s 넘게 이어진 구간마다, 시작 settle-s 뒤 (조향이 다 꺾인 뒤) 의 IMU wz / odom v 로 실제 곡률.
출력: 구간별 (시각, 방향, 길이, 속도, 실제 곡률·반경) + 방향별 정리. 반경 = 1 / 곡률 [m] (차 기준점: limo_base odom = 차 중심).
"""
import argparse
import bisect

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while r.has_next():
        t, data, ts = r.read_next()
        out[t].append((ts * 1e-9, deserialize_message(data, get_message(types[t]))))
    return out


def med(a):
    a = sorted(a)
    return a[len(a) // 2] if a else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--kmax', type=float, default=2.16)
    ap.add_argument('--min-s', type=float, default=1.0)
    ap.add_argument('--settle-s', type=float, default=0.6)
    a = ap.parse_args()
    d = read(a.bag, ['/cmd_vel', '/imu', '/odom'])
    cmd, imu, odom = d['/cmd_vel'], d['/imu'], d['/odom']
    it, ot = [x[0] for x in imu], [x[0] for x in odom]
    t0 = cmd[0][0]
    runs, cur = [], None
    for t, m in cmd:
        v, w = m.linear.x, m.angular.z
        sat = abs(v) > 0.05 and abs(w / v) > a.kmax
        sgn = 1 if w * v > 0 else -1
        if sat and cur is not None and cur['sgn'] == sgn:
            cur['t1'] = t
        else:
            if cur is not None and cur['t1'] - cur['t0'] >= a.min_s:
                runs.append(cur)
            cur = {'t0': t, 't1': t, 'sgn': sgn} if sat else None
    if cur is not None and cur['t1'] - cur['t0'] >= a.min_s:
        runs.append(cur)
    print(f'최대 조향 구간 (명령 |k| > {a.kmax} 가 {a.min_s} s 넘게): {len(runs)} 개')
    print(f'{"시작[s]":>7} {"방향":<4} {"길이":>5} {"v":>5} {"실제k":>6} {"반경[m]":>7} {"샘플":>4}')
    by = {1: [], -1: []}
    for r in runs:
        i0, i1 = bisect.bisect_left(it, r['t0'] + a.settle_s), bisect.bisect_left(it, r['t1'])
        ks, vs = [], []
        for t, m in imu[i0:i1]:
            j = min(bisect.bisect_left(ot, t), len(odom) - 1)
            v = odom[j][1].twist.twist.linear.x
            if abs(v) > 0.05:
                ks.append(m.angular_velocity.z / v)
                vs.append(v)
        if not ks:
            continue
        k = med(ks)
        by[r['sgn']] += ks
        print(f'{r["t0"] - t0:7.1f} {"왼" if r["sgn"] > 0 else "오른":<4} {r["t1"] - r["t0"]:5.1f} {med(vs):5.2f} {k:+6.2f} '
              f'{1 / abs(k) if k else float("inf"):7.2f} {len(ks):4d}')
    for s, name in ((1, '왼쪽'), (-1, '오른쪽')):
        ks = sorted(abs(k) for k in by[s])
        if ks:
            print(f'{name}: 최대 조향 때 실제 곡률 중앙값 {med(ks):.2f} → 반경 {1 / med(ks):.2f} m, '
                  f'상위 10 % 곡률 {ks[int(len(ks) * 0.9)]:.2f} → 반경 {1 / ks[int(len(ks) * 0.9)]:.2f} m (샘플 {len(ks)})')
        else:
            print(f'{name}: 최대 조향 구간 없음')


if __name__ == '__main__':
    main()
