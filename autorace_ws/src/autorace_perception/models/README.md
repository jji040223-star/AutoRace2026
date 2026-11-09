`autorace_yolov8n.pt` = https://github.com/mark2455/2026_Autorace `yolo-training/models/best.pt`
(커밋 5720429, 2026-10-29 학습, YOLOv8n 640, 클래스 crosswalk·parking·slow·speed_bump·turn_a·turn_b, 테스트 mAP50 0.995).
데이터: Roboflow autorace-sospj (CC BY 4.0). Ultralytics 이용 조건은 별도.

`sign_branch_A.png`, `sign_branch_B.png` = M2 갈림길 표지판 그림 (규정집 VII장 그림에서 잘라 낸 것, `~/LimoIsaacSIM/autorace_2026/data/mission/textures/` 와 같은 파일).
yolo_detector 의 갈림길 표지판 보조 판별 (`branch_sign.py`) 이 A/B 템플릿으로 쓴다. 실제 대회 표지판 그림이 다르면 이 두 장을 바꾼다.

`traffic_light_yolov8n.pt` = M1 신호등 YOLOv8n 640, 클래스 `red_light`·`yellow_light`·`green_light` (2026-11-02). 시뮬 영상 293 장
(빨강 77·노랑 77·초록 72·배경 67 — 광고 모니터 화면 150 장 + 방해 그림 모니터 150 장, `ros2 run autorace_sim light_dataset` 로 수집, 박스 = 신호등 몸체)
으로 위 `autorace_yolov8n.pt` 에서 시작해 80 epoch (색상 hsv_h 증강 끔). 검증 44 장 mAP50 0.995. 데이터: `~/datasets/traffic_light_sim`.
시뮬 렌더 전용이라 실차 영상으로 다시 학습할 것.

`traffic_light_real_yolov8n.pt` = M1 실차 신호등 YOLOv8n 640, 클래스 `green sign`·`red sign`·`yellow sign` (2026-11-06).
실물 장난감 신호등을 휴대폰으로 찍은 영상 3 개 (IMG_3941 빨강 / 3942 초록 / 3943 노랑) 에서 515 장, 박스 = 정면 3구 몸체만
(옆면 보행 신호·다른 표지판은 라벨 없음). 데이터: AutoRace2026 저장소 `yolo/traffic_light`. yolov8n.pt 에서 80 epoch.
drive.launch 가 실차 (use_sim_time:=false) 에서 자동으로 쓰고, traffic_light 노드에 `config/traffic_light_real.yaml` 을 덮어쓴다.
다른 모델로 바꾸려면 `light_model:=경로.pt` (라벨이 다르면 `light_params:=설정.yaml`).
