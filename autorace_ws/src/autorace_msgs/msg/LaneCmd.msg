# /mission/lane_cmd — mission_manager → lane_follower 지시 (v3 문서 2장).
# lane_follower 가 쓰는 것: max_speed, stop, reset_window. 나머지는 이후 미션용 자리 (아직 무시됨).
float32 max_speed        # [m/s] 속도 상한. 0 이하 = 상한 없음 (lane_follower 기본 속도)
bool stop                # true = 바로 정지 (v = 0, ω = 0, 램프 없이) — M3·M1 정지선
string follow            # "both" | "left" | "right"          — M2 갈림길 (미구현)
float32 target_offset_m  # 차로 중심에서 옆으로 [m], + = 왼쪽  — M7 차선 변경 (미구현)
string param_set         # "" | "zigzag"                      — M6 (미구현)
bool use_white           # 흰색 차선 검출                       — M7 (미구현)
bool reset_window        # true 로 바뀌는 순간 슬라이딩 윈도우 다시 시작 — M7 회피 뒤 차선 복귀 (M8·M4 도)
