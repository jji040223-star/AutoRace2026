# 리모 (LIMO Pro, Jetson) 실행 스크립트 사본

리모의 `~/AutoRace2026_sim2d/` 에 있는 파일을 그대로 복사해 둔 것이다 (11-12 기준).
리모를 다시 설치하면 이 폴더의 파일을 `~/AutoRace2026_sim2d/` 에 다시 놓는다.

## 고정 기준 (11-12 ack56 완주)

- 코드: 브랜치 `final` (= 태그 `real-best-1112` 에서 기록·시뮬 도구를 뺀 것)
- 설정: `autorace_ws/src/autorace_bringup/config/real_limo.yaml`
- 결과: 출발선부터 M5·M6·M3·M7·M2·M8·M1·M4 (주차 + 탈출) 까지 혼자 완주. 주행 전체 CPU 평균 72 %, 최대 88 %.

## 순서

1. 센서를 켠다 (EKF·점구름·IR 은 끈다).

   ```bash
   source /opt/ros/humble/setup.bash && source ~/wego_ws/install/setup.bash
   ros2 launch ~/AutoRace2026_sim2d/teleop_lite_launch.py
   ```

   `/limo_status` 의 `control_mode` 가 **1** 이어야 한다. 리모컨을 쓰면 3 이 되고, 3 이면 차가 명령을 무시한다.

2. 주행한다. 대회에서는 `NOBAG=1` 로 녹화를 끈다 (녹화가 카메라 압축으로 CPU 를 약 60 % 더 쓴다).

   ```bash
   cd ~/AutoRace2026_sim2d
   NOBAG=1 LIGHT=yolo MP=full.yaml setsid nohup bash run_drive.sh <이름> \
     light_model:=$HOME/AutoRace2026_sim2d/sign_best2_1110.pt \
     light_params:=$HOME/AutoRace2026_sim2d/light_best.yaml > /tmp/<이름>.out 2>&1 < /dev/null &
   ```

   CPU 를 기록하려면 주행 폴더를 넘겨 `bash cpu_log.sh <주행 폴더>/cpu` 를 같이 켠다.

3. 멈춘다. 남은 노드가 0 개가 될 때까지 다시 실행한다.

   ```bash
   bash ~/AutoRace2026_sim2d/stop_drive.sh
   pkill -f cpu_log.sh; pkill -f "tegrastats --interval"
   ```

## 파일

| 파일 | 내용 |
|---|---|
| `teleop_lite_launch.py` | wego `teleop_launch.py` 복사본. EKF 를 빼고 카메라 점구름·IR 을 끔 (우리 노드는 쓰지 않음) |
| `run_drive.sh` | 주행 시작. `NOBAG=1` 이면 녹화하지 않음 |
| `stop_drive.sh` | 주행 즉시 멈춤 |
| `env.sh` | CUDA·TensorRT·dl_env 환경 + 워크스페이스 |
| `merge_params.py` | `real_limo.yaml` 에 속도 (`SPEED`) 를 넣어 주행 폴더에 저장 |
| `full.yaml` | 미션 설정 (`MP=full.yaml`, 모든 미션) |
| `light_best.yaml` | 신호등 설정 |
| `cpu_log.sh` | 2 초마다 top + tegrastats 기록 |
| `bevsnap.py` | 지금 카메라 한 장과 `/lane/debug` (BEV) 한 장을 PNG 로 저장 |

| `sign_best2_1110.pt` | 신호등 YOLO 모델 (`light_model:=`, 팀원 11-10 통합 모델). 표지판 모델은 autorace_perception 패키지 안 |
