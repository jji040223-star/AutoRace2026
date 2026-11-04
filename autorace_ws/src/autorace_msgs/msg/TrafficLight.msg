# /perception/traffic_light — traffic_light 노드 (M1). 처리한 영상마다 하나
std_msgs/Header header
string color             # "red" | "yellow" | "green" | "none" (안 보임·애매)
float32 contrast         # 가장 밝은 칸 ÷ 두 번째 칸 밝기 (한 칸만 켜졌는지)
int32 x1                 # 고른 박스 (원본 영상 픽셀). 없으면 0
int32 y1
int32 x2
int32 y2
float32 dist_m           # 크기로 잰 카메라 → 신호등 거리 (실물 높이 0.205 m)
float32 expected_dist_m  # 정지선 (BEV 에서 본 뒤 오도메트리로 이어 감) 으로 잰 예상 거리, 모르면 -1
bool size_checked        # 예상 거리와 크기가 맞는지까지 확인한 판정 (발진·통과는 이것만 믿는다)
int32 candidates         # 들어온 박스 수
int32 rejected           # 필터로 버린 수
string reason            # 버린 이유 요약 / none 인 이유
