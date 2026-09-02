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

헬퍼는 `build_trt` / `install_trt`에 넣는다. 컨테이너 안에서 직접 할 때:

```bash
cd /workspaces/eclipse-test-2
colcon build --symlink-install \
  --packages-select eclipse_pkg_msgs eclipse_pkg tars_recovery_behaviors tars_tide_layer \
  --build-base build_trt --install-base install_trt
source install_trt/setup.bash
```

## 기동

`description`과 `autonomy`를 동시에 돌리지 말 것. 둘 다 `eclipse_test_controller`를 띄운다.

```bash
./scripts/run_jetson_description.sh     # GPS/IMU/EKF/모터
./scripts/run_jetson_autonomy.sh        # 제부 수위선 + Nav2
./scripts/run_jetson_autonomy.sh stop
```

기본 `keepout_site=jebu`. 워크스페이스 기본값은 이 리포 루트 (`ECLIPSE_WORKSPACE`로 덮어쓰기).

## 실차 숫자 (2026-09)

- 증속 G=2 (40/20), 플랫폼 상한 ≈ 0.779 m/s
- 높이 전류 클램프 4000 mA
- global costmap 4000 m @ 0.8 m
