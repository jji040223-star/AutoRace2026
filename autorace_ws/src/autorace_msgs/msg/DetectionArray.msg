# /perception/detections — yolo_detector 가 영상 한 장마다 하나 (검출이 없어도 빈 배열로 낸다)
std_msgs/Header header     # 원본 영상의 시각·프레임
uint32 image_width
uint32 image_height
Detection[] detections
