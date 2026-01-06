# Louvre Extension with ROS 2 Trash Detection

Isaac Sim extension for Louvre museum scene with trash detection and autonomous navigation.

## 📂 디렉토리 구조

```
louvre_extension_minimal/
├── exts/
│   └── louvre.scene.builder/      # Extension 코드
│       ├── extension.toml         # Extension 설정
│       └── louvre/scene/builder/
│           └── extension.py       # 메인 로직 (상대 경로 사용)
├── ros_nodes/
│   ├── trash_detection/           # ROS 2 쓰레기 탐지 패키지
│   │   ├── test_dect.py          # 쓰레기 탐지 시각화
│   │   ├── go_to_trash.py        # 자율 주행 노드
│   │   ├── setup.py              # ROS 2 패키지 설정
│   │   └── package.xml           # 패키지 메타데이터
│   └── louvre_navigation/         # ROS 2 네비게이션 패키지
│       ├── launch/               # Launch 파일들
│       ├── params/               # 파라미터 설정
│       ├── maps/                 # 맵 파일
│       └── louvre_navigation/    # Python 노드들
├── assets/
│   ├── robots/
│   │   └── RidgebackFranka/      # 로봇 USD (ActionGraph 포함)
│   └── wheel/                    # Mecanum wheel USD
├── scenes/
│   └── lubre.glb                 # Louvre 환경 (텍스처 포함)
├── isaac-sim-lou.sh              # Isaac Sim 실행 스크립트
└── README.md                     # 이 파일
```

## 🚀 실행 방법

### 1. Isaac Sim 경로 설정

환경변수로 Isaac Sim 경로를 설정하거나, 기본값 `~/isaacsim` 사용:

```bash
# 방법 1: 환경변수 설정
export ISAAC_SIM_PATH=/path/to/your/isaacsim

# 방법 2: 기본값 사용 (~/isaacsim에 설치된 경우)
# 별도 설정 불필요
```

### 2. Isaac Sim 실행 (간편 스크립트)

```bash
cd /path/to/louvre_extension_minimal
./isaac-sim-lou.sh
```

### 1. Isaac Sim Extension 실행

```bash
# Isaac Sim 실행 (간편 스크립트)
cd /path/to/louvre_extension_minimal
./isaac-sim-lou.sh
```

또는 직접 실행:

```bash
cd /path/to/isaac-sim
./isaac-sim.sh --ext-folder /path/to/louvre_extension_minimal/exts
```

**Extension 활성화**:
- **Window** → **Extensions** → 검색: `louvre`
- **ENABLED** 토글
- **Build Louvre Scene** 버튼 클릭 → **Play** 버튼

### 2. ROS 2 노드 설치 및 실행

```bash
# ROS 2 워크스페이스에 패키지 복사
cd ~/ros2_ws/src
cp -r /path/to/louvre_extension_minimal/ros_nodes/trash_detection .
cp -r /path/to/louvre_extension_minimal/ros_nodes/louvre_navigation .

# 빌드
cd ~/ros2_ws
colcon build --packages-select trash_detection louvre_navigation
source install/setup.bash

# YOLO 모델 배치 (trash.pt 또는 trash1.pt)
# trash_detection/trash_detection/ 디렉토리에 모델 파일 복사

# Terminal 1: SLAM (맵 생성)
ros2 launch louvre_navigation louvre_slam.launch.py

# Terminal 2: 쓰레기 탐지 시각화
ros2 run trash_detection test_dect

# Terminal 3: 자율 주행
ros2 run trash_detection go_to_trash
```

**Navigation 사용 시**:
```bash
# SLAM 대신 Navigation 실행 (저장된 맵 필요)
ros2 launch louvre_navigation louvre_navigation.launch.py
```

## 📦 주요 기능

### Extension 기능
- ✅ Louvre 환경 로딩 (GLB with embedded textures)
- ✅ RidgebackFranka 로봇 추가 (ActionGraph 보존)
- ✅ Mecanum wheels, RealSense D455 카메라, LiDAR 센서
- ✅ 랜덤 쓰레기 객체 생성 (`usdz_only/` 디렉토리)
- ✅ Scene Save/Load (reference 방식으로 graph 보존)

### ROS 2 패키지 기능

#### trash_detection

**test_dect.py**:
- YOLO 기반 실시간 쓰레기 탐지
- `/rgb` 토픽 구독
- `/trash_detections` 토픽으로 결과 발행
- OpenCV 윈도우로 시각화

**go_to_trash.py**:
- YOLO로 쓰레기 탐지
- Depth camera로 거리 측정
- 가장 가까운 쓰레기로 자율 주행 (`/cmd_vel`)
- 탐지 없을 시 자동 탐색

#### louvre_navigation

**Launch files**:
- `louvre_slam.launch.py`: SLAM으로 맵 생성 (Cartographer)
- `louvre_navigation.launch.py`: Nav2 기반 자율 주행
- `louvre_tf.launch.py`: TF 변환 설정

**Nodes**:
- `laser_scan_merger.py`: 전후방 LiDAR 데이터 병합
- `depth_republisher.py`: Depth 이미지 재발행

## ⚙️ 파라미터

### go_to_trash 파라미터
- `detection_confidence`: 탐지 신뢰도 임계값 (default: 0.7)
- `camera_fov_h`: 카메라 수평 FOV (default: 69.4°)
- `min_y_ratio`: 이미지 상단 무시 비율 (default: 0.3)
- `linear_speed`: 최대 전진 속도 (default: 0.5 m/s)
- `angular_speed`: 회전 속도 게인 (default: 1.0 rad/s)

## 🏗️ 시스템 구조

```
Isaac Sim
  └─ Louvre Scene Builder Extension
       ├─ Scene Management
       ├─ Robot Spawning
       └─ Trash Spawning
          │
          └─ ROS 2 Bridge
               ├─ /rgb (sensor_msgs/Image)
               ├─ /depth (sensor_msgs/Image)
               ├─ /odom (nav_msgs/Odometry)
               ├─ /cmd_vel (geometry_msgs/Twist)
               ├─ /front_scan, /rear_scan (sensor_msgs/LaserScan)
               └─ /scan (merged LaserScan)
                    │
                    └─ ROS 2 Packages
                         ├─ trash_detection
                         │   ├─ test_dect (visualization)
                         │   └─ go_to_trash (trash navigation)
                         └─ louvre_navigation
                             ├─ SLAM (Cartographer)
                             ├─ Nav2 (autonomous navigation)
                             └─ Sensor fusion
```

## 🔧 이식성 (Portability)

**상대 경로를 사용하므로 어느 경로에 복사해도 동작합니다!**

- `extension.py`: `Path(__file__)` 기준 상대 경로 사용
- `isaac-sim-lou.sh`: `$SCRIPT_DIR` 기준 상대 경로 사용

## 📋 필수 요구사항

- NVIDIA Isaac Sim 2023.1.1 or later
- ROS 2 Humble
- Python 3.10
- YOLO 모델 파일 (`trash.pt` or `trash1.pt`)
- ultralytics 패키지

## 💡 팁

- 이 디렉토리만 복사하면 다른 컴퓨터에서도 즉시 실행 가능
- Isaac Sim과 인터넷 연결만 있으면 OK
- 폴더 이름 변경 가능 (경로 수정 불필요)
- 다른 사용자명에서도 그대로 동작
