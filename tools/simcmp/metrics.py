"""drive.log → 인식·M7 지표 (실차·시뮬 같은 기준). 사용: python3 metrics.py <drive.log> ..."""
import re, sys, json

OBS = re.compile(r'장애물 (\d+) 개: (.*)')
ONE = re.compile(r'앞 ([\d.]+) m 옆 ([+-][\d.]+)~([+-][\d.]+) 높이 ([\d.]+)')


def phase_of(line, cur):
    m = re.search(r'\[상태\] (\S+) → (\S+)', line)
    return m.group(2) if m else cur


def metrics(path):
    ph, seen = 'LANE', []
    r = dict(run=path.split('/')[-2], corner_lines=0, corner_tall=0, m7_lines=0, m7_tall=0, m7_obs_per_line=0.0,
             m7_entry_mem=None, m7_lane_changes=None, m7_len_m=None, back=0, pivot=0, blocked=0, overlap_mem=0,
             relocate=0, m7_done=False, parked=False, m7_t0=None, m7_t1=None, states=[])
    after_m3 = False
    for line in open(path, errors='ignore'):
        t = re.search(r'\[(\d{10}\.\d+)\]', line)
        t = float(t.group(1)) if t else None
        nph = phase_of(line, ph)
        if nph != ph:
            r['states'].append(nph)
            if nph == 'M3_GO':
                after_m3 = True
            if nph == 'M7_AVOID':
                m = re.search(r'기억한 장애물 (\d+) 개', line)
                r['m7_entry_mem'] = int(m.group(1)) if m else None
                r['m7_t0'] = t
            if ph == 'M7_AVOID':
                r['m7_done'] = True
                r['m7_t1'] = t
                m = re.search(r'차로 변경 (\d+) 번.*구간 ([\d.]+) m', line)
                if m:
                    r['m7_lane_changes'], r['m7_len_m'] = int(m.group(1)), float(m.group(2))
            if nph == 'DONE' and 'parking' in line:
                r['parked'] = True
            ph = nph
        m = OBS.search(line)
        if m and 'depth_obstacle' in line:
            obs = [tuple(map(float, o)) for o in ONE.findall(m.group(2))]
            tall = any(o[3] >= 0.45 for o in obs)
            if ph == 'LANE' and after_m3 and r['m7_t0'] is None:
                r['corner_lines'] += 1
                r['corner_tall'] += tall
            if ph == 'M7_AVOID':
                r['m7_lines'] += 1
                r['m7_tall'] += tall
                r['m7_obs_per_line'] += int(m.group(1))
        if ph == 'M7_AVOID':
            r['back'] += '[M7] 후진 끝' in line
            r['pivot'] += '[M7] 제자리 회전' in line
            r['blocked'] += '[M7] 막힘' in line
            r['overlap_mem'] += '[M7] 겹친 기억' in line
            r['relocate'] += '노란 선으로 위치' in line
    if r['m7_lines']:
        r['m7_obs_per_line'] = round(r['m7_obs_per_line'] / r['m7_lines'], 1)
    if r['m7_t0'] and r['m7_t1']:
        r['m7_s'] = round(r['m7_t1'] - r['m7_t0'], 1)
    r['states'] = ' '.join(s for s in r['states'] if s in ('M3_STOP', 'M7_AVOID', 'M2_BRANCH', 'M4_PARKING', 'DONE'))
    for k in ('m7_t0', 'm7_t1'):
        r.pop(k)
    return r


if __name__ == '__main__':
    for p in sys.argv[1:]:
        print(json.dumps(metrics(p), ensure_ascii=False))
