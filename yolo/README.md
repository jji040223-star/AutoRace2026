# YOLO

AutoRace 2026용 YOLO 객체 인식 데이터셋 모음입니다.

| 데이터셋 | 클래스 | 이미지 수 | 폴더 |
|---|---|---|---|
| 신호등 (Traffic Light) | `green sign`, `red sign`, `yellow sign` | 515 | [`traffic_light/`](traffic_light/) |

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
└── traffic_light_yolo.zip.part02    # 19MB
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
