# /lane/state — lane_follower → mission_manager, 처리한 영상마다 하나 (v3 문서 2장)
std_msgs/Header header       # 원본 영상의 시각·프레임
bool found                   # 차선을 찾았는지
float32 offset               # [-1, 1], + = 차선 중심이 화면 오른쪽
float32 curvature            # [-1, 1]
float32 center_offset_m      # 차 → 차선 가운데 옆 거리 [m], + = 왼쪽 (offset 을 BEV 축척으로 바꾼 것)
float32 lane_heading_rad     # 차 방향 기준 차선 방향 [rad], + = 왼쪽으로 돈 방향 (잡은 선의 BEV 기울기). 선을 못 잡으면 0
int8 yellow_side             # 이번 프레임 잡은 노란 선: 2 = 양쪽, -1 = 왼쪽만, +1 = 오른쪽만, 0 = 없음
                             #   3차선에서 차로 번호 (v3): 왼쪽만 = 맨 왼쪽 차로, 오른쪽만 = 맨 오른쪽, 없음 = 가운데
float32 lane_width_px        # 학습된 차로 폭 (BEV px, 정상 ≈ 192)
float32 width_near_px        # 이번 프레임 두 경계 사이 폭 — BEV 맨 아래 / 맨 위. 양쪽 선이 다 안 잡히면 0 (M6 지그재그 판단)
float32 width_far_px
float32 line_resid_px        # 슬라이딩 윈도우 곡선 피팅 잔차 [px] — 톱니 모양 지그재그 선이면 커진다 (M6). 선을 못 잡으면 0
float32 stop_line_dist_m     # 카메라 → 정지선(가까운 쪽 끝) 바닥 거리 [m]. 없으면 -1 (BEV 0.37–0.75 m 안에서만 보임)
float32 stop_line_cov        # 정지선이 차로 폭을 덮은 비율
