# eclipse-summit

TARS 갯벌 구조 로봇 ROS 2 Humble 스택. 이 트리만 있으면 기존 Jetson 이미지 위에 빌드하고 기동할 수 있다.

## 패키지

- `eclipse_pkg` — 노드, launch, Nav2 yaml, BT
- `eclipse_pkg_msgs` — `PresentCurrent`, `GpsGoal`
- `tars_recovery_behaviors` — Stall/Slip/Costmap 분기, SetHeight
- `tars_tide_layer` — 수위선 lethal costmap 레이어

## 한 번 빌드

Jetson에서 이미지 `eclipse-test-2:humble`가 있어야 한다. 없으면 `docker/jetson_test2.Dockerfile`로 만든다 (베이스 `eclipse-v2:humble`).

```bash
./scripts/build_jetson_workspace.sh
```

컨테이너 안에서 직접 할 때:

```bash
cd /workspaces/eclipse-test-2
colcon build --symlink-install \
  --packages-select eclipse_pkg_msgs eclipse_pkg tars_recovery_behaviors tars_tide_layer \
  --build-base build_trt --install-base install_trt
source install_trt/setup.bash
```

## 기동

```bash
./scripts/run_jetson_autonomy.sh          # 제부 수위선 기본
./scripts/run_jetson_autonomy.sh tiles    # 전국 GPS 타일
ENABLE_YOLO_TRT=true ./scripts/run_jetson_autonomy.sh
./scripts/run_jetson_autonomy.sh stop
```

기본 `keepout_site=jebu`. YOLO sidecar는 `imgsz=640` (merged5 엔진). 조사 격자 노드는 이 트리에 없다.

## 실차 숫자 (2026-09)

- 증속 G=2 (40/20), 플랫폼 상한 ≈ 0.779 m/s
- 높이 전류 클램프 4000 mA
- global costmap 4000 m @ 0.8 m
