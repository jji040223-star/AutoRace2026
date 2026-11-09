"""묶음 결과 한 줄씩: python3 summarize.py <접두사>  (out/<접두사>N_* 10 개)"""
import glob, json, re, subprocess, sys
pre = sys.argv[1]
runs = sorted([p for p in glob.glob(f'out/{pre}[0-9]*_*') if re.fullmatch(rf'{pre}\d+_(default|mirror|rand|gen)', p.split('/')[-1])],
              key=lambda p: int(re.search(rf'{pre}(\d+)_', p).group(1)))
ok = 0
for d in runs:
    n = d.split('/')[-1]
    e = open(f'{d}/eval.log', errors='ignore').read(); dl = open(f'{d}/drive.log', errors='ignore').read()
    sl = open(f'{d}/sim.log', errors='ignore').read()
    m = re.search(r'SIM2D_RESULT (.*)', e); j = json.loads(m.group(1)) if m else {}
    mt = subprocess.run(['python3', 'metrics.py', f'{d}/drive.log'], capture_output=True, text=True).stdout
    mj = json.loads(mt) if mt.strip() else {}
    m8 = re.search(r'라바콘 통과: ([\d.]+) s, [\d.]+ m, 후진 (\d+)', dl)
    pk = re.findall(r'네 모서리 여유 \(([^)]*)\)', dl)
    mins = min(float(x) for x in re.findall(r'[+-][\d.]+', pk[-1])) if pk else None
    lo = re.findall(r'차선 이탈 \(규정[^—]*— 차 \(([\d.]+), ([\d.]+)\)', sl)
    st = re.findall(r'\[상태\] (\S+ → \S+)', dl)
    cr = ('③' if '깔때기 선 무시' in dl else '') + ('②' if '바퀴 규칙으로' in dl else '')
    asst = '보조' if '코너 보조' in dl else ''
    done = j.get('reason') == 'parking DONE' and not j.get('collisions') and mj.get('m2_ok') is not False
    ok += done
    print(f"{n:10s} {j.get('reason','-'):12s} {round(j.get('sim_s',0))}s 충돌{list(j.get('collisions',{}))} 콘{len(j.get('knocked',[]))} "
          f"이탈{len(lo)} {[(round(float(a),1), round(float(b),1)) for a, b in lo][:3]} | M7 {mj.get('m7_s')}s 막힘{mj.get('blocked')} "
          f"멈춤{dl.count('멈춤. 장애물')} {cr} | M2 {mj.get('branch_sign')}→{mj.get('branch_went')} | "
          f"M8 {m8.group(1)+'s/후진'+m8.group(2) if m8 else '-'} {asst} | 주차 {mins} | 끝 {st[-1] if st else '-'}")
print(f'완주 (주차·탈출, 충돌 0, 갈림길 맞음): {ok}/{len(runs)}')
