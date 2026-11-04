# YOLO 검출 하나. 좌표는 원본 영상 픽셀 (DetectionArray 의 image_width × image_height 기준)
string label        # 모델 클래스 이름: crosswalk, parking, slow, speed_bump, turn_a, turn_b
float32 confidence
int32 x1
int32 y1
int32 x2
int32 y2
