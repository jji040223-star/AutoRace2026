# 깨진 rosbag2 sqlite 에서 메시지 행을 페이지 단위로 직접 읽어 /odom (Odometry) 과 /imu (Imu) 를 복구
import sys, struct, pickle
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
p = sys.argv[1]; out = sys.argv[2]
data = open(p, 'rb').read(); PS = 4096
def varint(b, i):
    v = 0
    for k in range(9):
        c = b[i + k]
        if k == 8: return (v << 8) | c, i + 9
        v = (v << 7) | (c & 0x7f)
        if c < 0x80: return v, i + k + 1
odo, imu = [], []
for pg in range(len(data) // PS):
    off = pg * PS; h = off + (100 if pg == 0 else 0)
    if data[h] != 0x0D: continue
    n = struct.unpack('>H', data[h + 3:h + 5])[0]
    for c in range(n):
        try:
            cp = off + struct.unpack('>H', data[h + 8 + 2 * c:h + 10 + 2 * c])[0]
            plen, i = varint(data, cp); rowid, i = varint(data, i)
            if plen > PS - 35: continue           # overflow (영상 등) 은 건너뜀
            rec = data[i:i + plen]
            hl, j = varint(rec, 0); types = []
            while j < hl:
                t, j = varint(rec, j); types.append(t)
            if len(types) != 4: continue
            pos = hl; vals = []
            for t in types:
                if t == 0: vals.append(None); continue
                if 1 <= t <= 6:
                    sz = [0, 1, 2, 3, 4, 6, 8][t]; vals.append(int.from_bytes(rec[pos:pos + sz], 'big', signed=True)); pos += sz
                elif t >= 12 and t % 2 == 0:
                    sz = (t - 12) // 2; vals.append(rec[pos:pos + sz]); pos += sz
                else:
                    vals = None; break
            if not vals or not isinstance(vals[3], bytes): continue
            ts, blob = vals[2], vals[3]
            if 600 < len(blob) < 800:
                try:
                    m = deserialize_message(blob, Odometry)
                    if m.header.frame_id == 'odom':
                        q = m.pose.pose.orientation; odo.append((ts / 1e9, m.pose.pose.position.x, m.pose.pose.position.y, 2 * __import__('math').atan2(q.z, q.w), m.twist.twist.linear.x)); continue
                except Exception: pass
            if 250 < len(blob) < 400:
                try:
                    m = deserialize_message(blob, Imu)
                    if 'imu' in m.header.frame_id: imu.append((ts / 1e9, m.angular_velocity.z))
                except Exception: pass
        except Exception:
            continue
odo.sort(); imu.sort()
pickle.dump({'odom': odo, 'imu': imu}, open(out, 'wb'))
print(len(odo), len(imu), odo[0][0] if odo else None, odo[-1][0] if odo else None)
