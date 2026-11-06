# YOLO

AutoRace 2026용 YOLO 객체 인식 데이터셋 모음입니다.

| 데이터셋 | 클래스 | 이미지 수 | 폴더 |
|---|---|---|---|
| 신호등 (Traffic Light) | `green sign`, `red sign`, `yellow sign` | 515 | [`traffic_light/`](traffic_light/) |

---

## 실차 신호등 모델 갈아끼우기 (LIMO Pro)

시뮬용 신호등 모델 (`traffic_light_yolov8n.pt`, 클래스 `red_light`·`yellow_light`·`green_light`) 은 Isaac 렌더 영상으로만 학습해서
실물 신호등을 잘 못 잡는다. 아래 데이터셋으로 학습한 **실차 모델 `traffic_light_real_yolov8n.pt`** 를 넣었다.

### 바뀐 것
| 파일 (`autorace_ws/src/…`) | 내용 |
|---|---|
| `autorace_perception/models/traffic_light_real_yolov8n.pt` | **새 실차 모델**. YOLOv8n 640, 클래스 `green sign`·`red sign`·`yellow sign`, 80 epoch |
| `autorace_perception/config/traffic_light_real.yaml` | **새 파일**. 실차일 때 `traffic_light` 노드 설정을 덮어쓴다: `color_labels` = 새 라벨, 박스 세로:가로 `aspect` 2.5–5.0 |
| `autorace_mission/launch/drive.launch.py` | 인자 `light_model` (`auto`·`sim`·`real`·`.pt` 경로) 와 `light_params` 추가 |
| `autorace_perception/config/traffic_light_yolo.yaml`, `models/README.md` | 설명만 고침 |

- `light_model:=auto` (기본): 시뮬이면 `sim` 모델, 실차 (`use_sim_time:=false`) 면 `real` 모델 + `traffic_light_real.yaml`.
  **시뮬 동작은 그대로**다.
- 왜 설정도 바꿔야 하나: 실차 박스는 정면 3구 몸체만 감싸서 세로:가로가 3.0–4.1 로 홀쭉하다.
  기존 시뮬 설정 (1.4–2.1) 으로는 박스가 전부 버려진다. 라벨 이름도 다르다.
- 같은 변경이 저장소의 두 워크스페이스 zip (`autorace_ws_B_20261102_*.zip`) 에도 들어 있다. 새로 푸는 PC 는 아래 설치가 필요 없다.

### 이미 `~/autorace_ws` 가 있는 LIMO Pro·다른 PC 에 넣기
```bash
git clone https://github.com/jji040223-star/AutoRace2026.git     # 이미 받았으면: cd AutoRace2026 && git pull
bash AutoRace2026/yolo/traffic_light/limo/install_real_light.sh  # 워크스페이스가 다른 곳이면 뒤에 경로
```
- 바뀐 파일을 워크스페이스에 복사한다. 덮어쓰는 파일은 `*.bak_날짜` 로 남긴다.
- 그다음 `autorace_perception`·`autorace_mission` 만 다시 빌드한다. 여러 번 돌려도 된다.

LIMO 에 인터넷이 없으면 다른 PC 에서 받아서 넘긴다:
```bash
scp -r AutoRace2026/yolo/traffic_light/limo agilex@<LIMO_IP>:~/light_real     # 사용자명·IP 는 LIMO 에 맞게
ssh agilex@<LIMO_IP> 'bash ~/light_real/install_real_light.sh'
```

### 실행·확인 (LIMO)
```bash
ros2 launch autorace_mission drive.launch.py use_sim_time:=false enable:=false   # 인식만 (바퀴 정지)
#  로그: [drive] 신호등 모델 (real): …/traffic_light_real_yolov8n.pt + …/traffic_light_real.yaml
ros2 topic echo /perception/traffic_light                                        # 색
ros2 run rqt_image_view rqt_image_view /perception/traffic_light_debug           # 고른 박스 초록, 버린 박스 빨강 + 이유
```

### 나중에 다른 모델로 바꿀 때 (빌드 없이)
```bash
ros2 launch autorace_mission drive.launch.py use_sim_time:=false light_model:=~/models/best.pt
# 라벨이 red/yellow/green sign 이 아니면 덮어쓸 yaml 도 준다:
#   traffic_light:
#     ros__parameters:
#       color_labels: ["빨강라벨", "노랑라벨", "초록라벨"]
ros2 launch autorace_mission drive.launch.py use_sim_time:=false light_model:=~/models/best.pt light_params:=~/models/my_light.yaml
```
- 없는 경로를 주면 launch 가 `신호등 YOLO 모델이 없음: …` 으로 바로 멈춘다.
- 실차에서 시뮬 모델을 쓰려면 `light_model:=sim`.

### 검증 (이 PC)
| 시험 | 결과 |
|---|---|
| test 52 장 (YOLO val) | mAP50 0.995, mAP50-95 0.981, 세 클래스 recall 1.0 |
| LIMO 카메라처럼 4:3 가로로 잘라 640×480 으로 줄인 130 장 | 130 장 모두 맞는 색 1 개 |
| 신호등이 없는 640×480 조각 117 장 | 오검출 0 |
| `traffic_light` 노드의 세 칸 밝기 확인 (YOLO 색과 다르면 버림) | 515 장 중 버려지는 것 0 |
| ultralytics 8.4.153 (LIMO 쪽 버전) 으로 모델 불러오기·추론 | 정상 |
| `drive.launch` `auto`/`sim`/잘못된 경로, 설치 스크립트를 빈 워크스페이스 복사본에 | 정상 |

**아직 안 한 것**
- 실제 LIMO 카메라 (Orbbec Dabai) 영상으로는 시험하지 않았다. 학습 영상은 휴대폰으로 찍은 것이다.
- 학습·시험 이미지가 같은 영상 3 개에서 나왔다. 그래서 위 점수는 실제보다 좋게 나왔을 수 있다.
- LIMO 로 찍은 영상에서 놓치거나 색을 틀리면, 그 영상을 데이터셋에 더해 다시 학습하는 것을 권장한다.
- `body_h_m` (몸체 높이 0.205 m) 는 시뮬 값 그대로다. 크기로 거리를 재서 정지선 예상 거리와 맞춰 보는 데 쓴다.
  대회 신호등을 재서 다르면 `traffic_light_real.yaml` 에 적는다.

---

## 신호등 데이터셋 (`traffic_light/`)

### 개요
- 형식: **YOLOv8** (Roboflow 내보내기와 같은 폴더 구조)
- 원본: 트랙 위 장난감 신호등을 촬영한 영상 3개 (IMG_3941 / 3942 / 3943.mov)
- 추출: 영상마다 6프레임에 1장씩 (약 5fps)
- 해상도: 1080 × 1920 (세로)

### 클래스

| ID | 이름 | 원본 영상 | 라벨 수 |
|---|---|---|---|
| 0 | `green sign` | IMG_3942.mov | 159 |
| 1 | `red sign` | IMG_3941.mov | 179 |
| 2 | `yellow sign` | IMG_3943.mov | 177 |

### 데이터 분할

| split | 이미지 수 |
|---|---|
| train | 360 |
| valid | 103 |
| test | 52 |

### 라벨링 기준
- 박스는 빨강·노랑·초록 **3구가 있는 정면 몸체만** 감쌉니다.
- 신호등 **옆면의 보행자(횡단보도) 신호는 제외**했습니다.
  - 보행 신호에는 노란 렌즈가 없어서, 노란 렌즈 위치를 기준으로 박스 폭을 정면 몸체에 맞췄습니다.
- 신호등 받침대는 박스에 넣지 않았습니다.
- 트랙의 다른 표지판(P, 서행, 삼각 표지판 등)은 라벨링하지 않았습니다.
- 가려졌거나 흔들림이 심해서 정면을 정확히 잡을 수 없는 프레임은 데이터셋에서 뺐습니다.
- 라벨링 방법: COCO 사전학습 YOLO11x로 신호등을 1차 검출하고, 색상 분석으로 박스를 정면 몸체에 맞게 보정한 뒤 육안으로 확인했습니다.

### 파일 구성
GitHub의 파일 크기 제한(100MB) 때문에 zip을 3조각으로 나눠 올렸습니다.

```
traffic_light/
├── data.yaml                        # 클래스 정의 (참고용, zip 안에도 들어 있음)
├── traffic_light_yolo.zip.part00    # 45MB
├── traffic_light_yolo.zip.part01    # 45MB
├── traffic_light_yolo.zip.part02    # 19MB
└── limo/                            # 실차 모델 갈아끼우기 (위 섹션)
    ├── install_real_light.sh
    └── files/src/…                  # 워크스페이스에 복사될 파일 (모델 .pt 포함)
```

### 합치고 압축 풀기

**Linux / macOS**
```bash
cd yolo/traffic_light
cat traffic_light_yolo.zip.part* > traffic_light_yolo.zip
sha256sum traffic_light_yolo.zip   # 아래 체크섬과 같은지 확인
unzip traffic_light_yolo.zip
```

**Windows (cmd)**
```cmd
cd yolo\traffic_light
copy /b traffic_light_yolo.zip.part00 + traffic_light_yolo.zip.part01 + traffic_light_yolo.zip.part02 traffic_light_yolo.zip
```

SHA-256: `df71f050fc85c5a8c47f09542ba889f1d7fa284f04c3d524a0b4fbf93b999bc7`

압축을 풀면 이런 구조가 나옵니다.
```
traffic_light_yolo/
├── data.yaml
├── README.dataset.txt
├── train/{images,labels}
├── valid/{images,labels}
└── test/{images,labels}
```

### 라벨 형식
이미지마다 같은 이름의 `.txt` 파일이 있고, 한 줄에 객체 하나씩 들어 있습니다(좌표는 0~1로 정규화).
```
<class_id> <x_center> <y_center> <width> <height>
1 0.823611 0.345548 0.065759 0.119577
```

### 학습 예시 (Ultralytics)
```bash
pip install ultralytics
yolo detect train data=traffic_light_yolo/data.yaml model=yolo11n.pt imgsz=640 epochs=100
```

### 주의
- 이미지는 영상 3개에서 뽑은 연속 프레임이라, train/valid/test 사이에 비슷한 장면이 섞여 있습니다. 실제 성능은 새로 찍은 영상으로 확인하는 것을 권장합니다.
- 영상마다 켜진 색이 하나로 고정되어 있어서, 클래스는 영상 단위로 붙였습니다.
