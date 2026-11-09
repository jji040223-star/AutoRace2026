"""M7 진입 직전 뎁스 관측 중 진입 조건 (앞 < 1.2, ±0.2 걸침, 폭 0.12~0.45, 높이 0.10~0.45) 을 만족한 것. 사용: python3 m7_trigger_check.py <drive.log> ..."""
import re, sys
ONE = re.compile(r'앞 ([\d.]+) m 옆 ([+-][\d.]+)~([+-][\d.]+) 높이 ([\d.]+)')
for p in sys.argv[1:]:
    hist = []
    for line in open(p, errors='ignore'):
        if 'depth_obstacle' in line and '장애물' in line:
            hist.append([tuple(map(float, o)) for o in ONE.findall(line)])
        if '→ M7_AVOID' in line:
            out = []
            for obs in hist[-3:]:
                t = [o for o in obs if o[0] < 1.2 and o[1] < 0.2 and o[2] > -0.2 and 0.12 <= o[2] - o[1] <= 0.45 and 0.10 <= o[3] <= 0.45]
                out.append(' / '.join(f'앞 {o[0]:.2f} 옆 {o[1]:+.2f}~{o[2]:+.2f} 높이 {o[3]:.2f}' for o in t) or '없음')
            print(p.split('/')[-2], '|', ' || '.join(out))
            break
