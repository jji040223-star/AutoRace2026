# 뎁스로 찾은 장애물 하나 (바닥 위로 솟은 점들의 묶음). base_link 기준 [m]: x 앞, y 왼쪽
float32 x_near     # 가장 가까운 점의 앞 거리 (보이는 앞면)
float32 x_far      # 가장 먼 점 (뒷면은 가려서 안 보인다 — 실제 길이보다 짧게 나옴)
float32 y_left     # 왼쪽 끝 (y 최대)
float32 y_right    # 오른쪽 끝 (y 최소)
float32 height     # 가장 높은 점 [m]
int32 points
