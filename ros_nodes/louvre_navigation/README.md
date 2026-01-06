# Louvre Navigation Package

Isaac Sim + ROS2 Humble 환경에서 Ridgeback Franka 로봇 네비게이션을 위한 패키지입니다.

## 📋 요구사항

- ROS2 Humble
- Nav2
- SLAM Toolbox
- Isaac Sim (ROS2 Bridge 활성화)

## 🚀 설치 방법

```bash
# 1. 패키지를 워크스페이스에 복사
cp -r louvre_navigation ~/humble_ws/src/

# 2. 의존성 설치
cd ~/humble_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 빌드
colcon build --packages-select louvre_navigation
source install/setup.bash
```

## 🎮 실행 방법

### 1. 네비게이션 실행 (기존 맵 사용)

```bash
# 터미널 1: Isaac Sim 실행 후 씬 로드 & Play

# 터미널 2: 네비게이션 실행
source ~/humble_ws/install/setup.bash
ros2 launch louvre_navigation louvre_navigation.launch.py
```

RViz에서 `2D Pose Estimate`로 초기 위치 설정 후, `Nav2 Goal`로 목표 지점 클릭하면 자율 주행합니다.

### 2. SLAM으로 새 맵 생성

맵이 변경되었을 때 새로운 맵을 생성하려면:

```bash
# 터미널 1: Isaac Sim 실행 후 씬 로드 & Play

# 터미널 2: SLAM 실행
source ~/humble_ws/install/setup.bash
ros2 launch louvre_navigation louvre_slam.launch.py

# 터미널 3: 키보드로 로봇 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 맵 스캔 완료 후, 터미널 4에서 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/louvre_map
```

저장된 맵 파일(`louvre_map.yaml`, `louvre_map.png`)을 `maps/` 폴더에 복사하세요.

## 📁 폴더 구조

```
louvre_navigation/
├── launch/
│   ├── louvre_navigation.launch.py  # 네비게이션 실행
│   ├── louvre_slam.launch.py        # SLAM 맵 생성
│   └── louvre_tf.launch.py          # TF만 실행
├── params/
│   ├── louvre_navigation_params.yaml  # Nav2 파라미터
│   └── slam_params.yaml               # SLAM Toolbox 파라미터
├── maps/
│   ├── louvre_map.yaml   # 맵 설정
│   └── louvre_map.png    # 맵 이미지
├── rviz2/
│   ├── louvre_navigation.rviz  # 네비게이션용 RViz 설정
│   └── louvre_slam.rviz        # SLAM용 RViz 설정
└── louvre_navigation/
    └── laser_scan_merger.py    # 듀얼 LiDAR 병합 노드
```

## 🔧 주요 설정

### 로봇 Footprint
```yaml
footprint: "[ [0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3] ]"
```

### LiDAR 토픽
- Front LiDAR: `/front_laser/scan`
- Rear LiDAR: `/rear_laser/scan`
- Merged: `/scan` (laser_scan_merger 노드가 병합)

### 속도 제한
- 최대 선속도: 0.5 m/s
- 최대 각속도: 1.0 rad/s

## ⚠️ 문제 해결

### base_link 없다고 뜰 때
Isaac Sim에서 TF가 퍼블리시되지 않는 경우입니다.

**확인 사항:**
1. Isaac Sim에서 **Play 버튼**을 눌렀는지 확인 (반드시 시뮬레이션 실행 중이어야 함)
2. 로봇에 **ROS2 Bridge**가 제대로 설정되어 있는지 확인

**TF 퍼블리시 확인:**
```bash
ros2 topic echo /tf
```

**수동으로 TF 퍼블리시 (임시 해결):**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

**Isaac Sim 로봇 설정 확인:**
- Articulation Root가 설정되어 있는지
- ROS2 Publish Transform 컴포넌트가 로봇에 추가되어 있는지
- Action Graph에서 TF Publisher 노드가 있는지

### TF 에러 발생 시
Isaac Sim에서 로봇의 odom → base_link TF가 정상적으로 퍼블리시되는지 확인:
```bash
ros2 run tf2_tools view_frames
```

### 맵이 안 보일 때
```bash
ros2 topic echo /map --once
```

### 로봇이 안 움직일 때
cmd_vel 토픽 확인:
```bash
ros2 topic echo /cmd_vel
```

## 📝 라이선스

Apache-2.0
