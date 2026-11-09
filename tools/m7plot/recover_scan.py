# 깨진 bag 에서 /scan (overflow 페이지 따라감) 을 지정 시각 근처만 복구
import sys, struct, pickle
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
p, out = sys.argv[1], sys.argv[2]; want = [float(x) for x in sys.argv[3:]]
data = open(p, 'rb').read(); PS = 4096; U = PS
def varint(b, i):
    v = 0
    for k in range(9):
        c = b[i + k]
        if k == 8: return (v << 8) | c, i + 9
        v = (v << 7) | (c & 0x7f)
        if c < 0x80: return v, i + k + 1
X = U - 35; M = ((U - 12) * 32 // 255) - 23
got = {}
for pg in range(len(data) // PS):
    off = pg * PS; h = off + (100 if pg == 0 else 0)
    if data[h] != 0x0D: continue
    n = struct.unpack('>H', data[h + 3:h + 5])[0]
    for c in range(n):
        try:
            cp = off + struct.unpack('>H', data[h + 8 + 2 * c:h + 10 + 2 * c])[0]
            plen, i = varint(data, cp); rowid, i = varint(data, i)
            if plen < 1000 or plen > 40000: continue
            K = M + (plen - M) % (U - 4); local = plen if plen <= X else (K if K <= X else M)
            payload = bytearray(data[i:i + local]); nxt = struct.unpack(">I", data[i + local:i + local + 4])[0] if plen > X else 0
            hl, j = varint(payload, 0); types = []
            while j < hl:
                t, j = varint(payload, j); types.append(t)
            if len(types) != 4 or not (1 <= types[2] <= 6): continue
            pos = hl
            for t in types[:2]:
                pos += [0, 1, 2, 3, 4, 6, 8][t] if 1 <= t <= 6 else 0
            sz = [0, 1, 2, 3, 4, 6, 8][types[2]]; ts = int.from_bytes(payload[pos:pos + sz], 'big', signed=True) / 1e9
            if not any(abs(ts - w) < 0.15 for w in want): continue
            while len(payload) < plen and nxt:
                o = nxt * 0 + (nxt - 1) * PS; nxt = struct.unpack('>I', data[o:o + 4])[0]; payload += data[o + 4:o + PS]
            payload = bytes(payload[:plen]); pos2 = pos + sz
            t3 = types[3]; bl = payload[pos2:pos2 + (t3 - 12) // 2]
            try:
                m = deserialize_message(bl, LaserScan)
                if len(m.ranges) > 100: got[ts] = (m.angle_min, m.angle_increment, list(m.ranges))
            except Exception: pass
        except Exception: continue
pickle.dump(got, open(out, 'wb')); print(len(got), sorted(got)[:5])
