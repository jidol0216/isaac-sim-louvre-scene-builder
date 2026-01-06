# Nav2 네비게이션 완벽 가이드 (Louvre Navigation)

Isaac Sim + ROS2 Humble 환경에서 Ridgeback Franka 로봇으로 Nav2 자율 주행하는 방법을 단계별로 설명합니다.

## 📋 목차

1. [시스템 구조](#시스템-구조)
2. [사전 준비](#사전-준비)
3. [SLAM으로 맵 생성하기](#slam으로-맵-생성하기)
4. [Nav2로 자율 주행하기](#nav2로-자율-주행하기)
5. [파라미터 설정](#파라미터-설정)
6. [문제 해결](#문제-해결)

---

## 시스템 구조

```
┌─────────────────────────────────────────────────────────────┐
│                       Isaac Sim                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Ridgeback    │  │ Front LiDAR  │  │ Rear LiDAR   │      │
│  │ Franka Robot │  │              │  │              │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│         │                  │                  │              │
│         └──────────────────┴──────────────────┘              │
│                            │                                 │
│                    ROS2 Bridge (Isaac Sim)                   │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 Topics                             │
│  /odom (nav_msgs/Odometry)                                   │
│  /cmd_vel (geometry_msgs/Twist)                              │
│  /front_laser/scan (sensor_msgs/LaserScan)                   │
│  /rear_laser/scan (sensor_msgs/LaserScan)                    │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│              louvre_navigation 패키지                         │
│                                                              │
│  ┌────────────────────────────────────────────────────┐     │
│  │  laser_scan_merger.py                              │     │
│  │  - /front_laser/scan + /rear_laser/scan            │     │
│  │  → /scan (360도 LiDAR)                             │     │
│  └────────────────────────────────────────────────────┘     │
│                             │                                │
│                             ▼                                │
│  ┌─────────────────┐  ┌─────────────────┐                   │
│  │   SLAM Mode     │  │   Nav2 Mode     │                   │
│  │  (맵 생성)       │  │  (자율 주행)     │                   │
│  └─────────────────┘  └─────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 사전 준비

### 1. 패키지 설치

```bash
# Nav2와 SLAM Toolbox 설치
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-teleop-twist-keyboard

# louvre_navigation 패키지 빌드
cd ~/ros2_ws
colcon build --packages-select louvre_navigation
source install/setup.bash
```

### 2. Isaac Sim 설정

1. **Extension 활성화**:
   - Window → Extensions → "louvre" 검색
   - Louvre Scene Builder 활성화

2. **씬 빌드**:
   - "Build Louvre Scene" 버튼 클릭
   - 또는 "Load Saved Scene" 버튼으로 저장된 씬 로드

3. **ROS2 Bridge 활성화**:
   - Isaac Utils → ROS2 Bridge
   - 자동으로 토픽이 퍼블리시됩니다

4. **Play 버튼** 클릭:
   - ▶️ 버튼을 눌러야 ROS2 토픽이 활성화됨!

### 3. 토픽 확인

```bash
# 터미널 새로 열어서
source /opt/ros/humble/setup.bash

# Odom 토픽 확인
ros2 topic echo /odom --once

# LiDAR 토픽 확인
ros2 topic echo /front_laser/scan --once
ros2 topic echo /rear_laser/scan --once

# TF 확인
ros2 run tf2_tools view_frames
# frames.pdf 생성됨 → odom → base_link 연결 확인
```

---

## SLAM으로 맵 생성하기

새로운 환경이거나 Louvre 씬이 변경된 경우, SLAM으로 맵을 먼저 생성해야 합니다.

### Step 1: Isaac Sim 실행

```bash
# 터미널 1
cd ~/louvre_extension_minimal
./isaac-sim-lou.sh

# Isaac Sim GUI에서:
# 1. Extension Manager → Louvre Scene Builder 활성화
# 2. "Build Louvre Scene" 또는 "Load Saved Scene"
# 3. ▶️ Play 버튼 클릭 (중요!)
```

### Step 2: SLAM 실행

```bash
# 터미널 2
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch louvre_navigation louvre_slam.launch.py
```

**실행되는 노드들**:
- `laser_scan_merger`: 전방/후방 LiDAR 병합 → `/scan`
- `slam_toolbox`: SLAM 알고리즘 (Async 모드)
- `rviz2`: SLAM 시각화
- `static_transform_publisher`: LiDAR TF 퍼블리시

### Step 3: 로봇 조종

```bash
# 터미널 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 키보드 조작:
# i: 전진
# ,: 후진
# j: 좌회전
# l: 우회전
# k: 정지
# q/z: 속도 증가/감소
```

**맵 스캔 팁**:
- 천천히 움직이세요 (속도가 너무 빠르면 맵이 왜곡됨)
- 모든 구역을 고르게 스캔하세요
- 같은 장소를 여러 번 지나가면 Loop Closure가 발생해 맵이 정밀해집니다
- RViz에서 실시간으로 맵이 생성되는 것을 확인하세요

### Step 4: 맵 저장

```bash
# 터미널 4 (맵 스캔 완료 후)
cd ~
ros2 run nav2_map_server map_saver_cli -f louvre_map

# 생성된 파일:
# - louvre_map.yaml: 맵 메타데이터
# - louvre_map.pgm: 맵 이미지
```

### Step 5: 맵 파일 복사

```bash
# 생성된 맵을 패키지로 복사
cp ~/louvre_map.yaml ~/ros2_ws/src/louvre_navigation/maps/
cp ~/louvre_map.pgm ~/ros2_ws/src/louvre_navigation/maps/louvre_map.png

# 다시 빌드 (맵 파일이 install 폴더로 복사됨)
cd ~/ros2_ws
colcon build --packages-select louvre_navigation
source install/setup.bash
```

**맵 확인**:
```bash
# 맵 이미지 열기
eog ~/ros2_ws/install/louvre_navigation/share/louvre_navigation/maps/louvre_map.png
```

---

## Nav2로 자율 주행하기

맵이 준비되었으면 Nav2로 자율 주행할 수 있습니다.

### Step 1: Isaac Sim 실행

```bash
# 터미널 1
cd ~/louvre_extension_minimal
./isaac-sim-lou.sh

# Isaac Sim에서:
# 1. "Load Saved Scene" (또는 "Build Louvre Scene")
# 2. ▶️ Play 버튼 클릭
```

### Step 2: Nav2 실행

```bash
# 터미널 2
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch louvre_navigation louvre_navigation.launch.py
```

**실행되는 노드들**:
- `laser_scan_merger`: LiDAR 병합
- `map_server`: 저장된 맵 로드
- `amcl`: 위치 추정 (Adaptive Monte Carlo Localization)
- `bt_navigator`: 행동 트리 기반 네비게이션
- `planner_server`: 경로 계획
- `controller_server`: 경로 추종 (DWB controller)
- `recoveries_server`: 복구 행동 (회전, 후진 등)
- `lifecycle_manager`: 노드 생명주기 관리
- `rviz2`: 네비게이션 시각화

### Step 3: 초기 위치 설정 (Localization)

RViz가 열리면:

1. **툴바에서 "2D Pose Estimate" 버튼** 클릭
2. **맵에서 로봇의 현재 위치를 클릭**하고 **드래그**하여 방향 설정
3. 초록색 화살표 구름(파티클)이 로봇 주변에 모이는지 확인
4. 파티클이 수렴하지 않으면 다시 시도

**팁**:
- Isaac Sim에서 로봇의 실제 위치를 확인하세요
- 맵에서 대략적인 위치를 찾으세요
- 방향이 중요합니다 (화살표 방향 = 로봇이 바라보는 방향)

### Step 4: 목표 지점 설정

1. **툴바에서 "Nav2 Goal" 버튼** 클릭
2. **맵에서 목표 지점을 클릭**하고 **드래그**하여 도착 방향 설정
3. 로봇이 자동으로 경로를 계획하고 이동 시작!

**RViz에서 확인할 것**:
- 파란색 선: Global Path (전체 경로)
- 초록색 선: Local Path (지역 경로, 실시간 조정)
- 빨간색 점: Costmap (장애물)
- 초록색 화살표 구름: AMCL 파티클 (위치 추정)

### Step 5: 네비게이션 중 동작

**로봇이 이동하는 동안**:
- 장애물을 자동으로 회피합니다
- 경로가 막히면 재계획합니다
- 갇히면 Recovery Behavior가 실행됩니다 (회전, 후진)

**목표 취소**:
- RViz 하단의 "Cancel Navigation" 버튼 클릭
- 또는 터미널에서: `Ctrl+C`로 노드 종료

**새로운 목표 설정**:
- 이동 중에도 "Nav2 Goal"로 새 목표를 설정할 수 있습니다
- 로봇이 즉시 새 경로로 재계획합니다

---

## 파라미터 설정

### 주요 파라미터 파일

#### 1. `louvre_navigation_params.yaml` (Nav2)

**로봇 Footprint** (크기):
```yaml
robot_radius: 0.5  # 원형 근사 반경 (미사용)
footprint: "[ [0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3] ]"  # 직사각형
# 단위: meter
# [x, y] 좌표로 로봇 윤곽 정의
```

**속도 제한**:
```yaml
max_vel_x: 0.5          # 최대 전진 속도 (m/s)
min_vel_x: -0.3         # 최대 후진 속도 (m/s)
max_vel_theta: 1.0      # 최대 회전 속도 (rad/s)
min_vel_theta: -1.0     # 최소 회전 속도 (rad/s)
```

**Controller (DWB)**:
```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  min_vel_x: 0.0
  max_vel_x: 0.5
  max_vel_theta: 1.0
  min_speed_xy: 0.0
  max_speed_xy: 0.5
  acc_lim_x: 2.5        # 가속도 제한
  acc_lim_theta: 3.2    # 각가속도 제한
  decel_lim_x: -2.5     # 감속도 제한
  decel_lim_theta: -3.2
```

**Costmap** (장애물 지도):
```yaml
local_costmap:
  width: 5                    # 5m x 5m 지역 맵
  height: 5
  resolution: 0.05            # 5cm 해상도
  update_frequency: 5.0       # 5Hz 업데이트
  publish_frequency: 2.0      # 2Hz 퍼블리시
  
global_costmap:
  width: 50                   # 전체 맵
  height: 50
  resolution: 0.05
  update_frequency: 1.0       # 1Hz 업데이트
```

**Inflation Layer** (장애물 팽창):
```yaml
inflation_layer:
  inflation_radius: 0.7       # 장애물로부터 70cm 거리 유지
  cost_scaling_factor: 3.0    # 비용 증가율
```

#### 2. `slam_params.yaml` (SLAM Toolbox)

```yaml
slam_toolbox:
  ros__parameters:
    # SLAM 모드
    mode: mapping                           # mapping / localization
    
    # 센서 설정
    scan_topic: /scan
    scan_queue_size: 10
    
    # 맵 해상도
    resolution: 0.05                        # 5cm/픽셀
    
    # Loop Closure (맵 보정)
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    
    # 성능 설정
    throttle_scans: 1                       # 1개 스캔마다 처리 (빠름)
    transform_timeout: 0.2
    
    # 맵 업데이트
    map_update_interval: 5.0                # 5초마다 맵 퍼블리시
```

---

## 문제 해결

### 1. "Waiting for odom->base_link transform"

**원인**: Isaac Sim에서 TF가 퍼블리시되지 않음

**해결**:
```bash
# TF 확인
ros2 run tf2_tools view_frames

# TF가 없으면:
# 1. Isaac Sim에서 Play 버튼을 눌렀는지 확인
# 2. ROS2 Bridge가 활성화되었는지 확인
# 3. 로봇 ActionGraph에서 "PublishTF" 노드가 있는지 확인
```

**임시 해결 (개발 중)**:
```bash
# 수동으로 TF 퍼블리시
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

### 2. "Map does not have an occupancy grid"

**원인**: 맵 파일 경로가 잘못되었거나 맵 파일이 없음

**해결**:
```bash
# 맵 파일 확인
ls ~/ros2_ws/install/louvre_navigation/share/louvre_navigation/maps/

# 맵이 없으면 SLAM으로 다시 생성
ros2 launch louvre_navigation louvre_slam.launch.py
```

### 3. "No laser scan received"

**원인**: LiDAR 토픽이 퍼블리시되지 않음

**해결**:
```bash
# LiDAR 토픽 확인
ros2 topic list | grep scan

# /front_laser/scan, /rear_laser/scan 있는지 확인
ros2 topic echo /front_laser/scan --once

# 없으면 Isaac Sim에서:
# 1. LiDAR 센서가 추가되었는지 확인
# 2. Play 버튼을 눌렀는지 확인
```

### 4. "AMCL cannot localize"

**원인**: 초기 위치가 잘못 설정되었거나 맵과 환경이 다름

**해결**:
1. **2D Pose Estimate를 다시 설정**:
   - 로봇의 정확한 위치와 방향을 설정하세요
   - 파티클 구름이 로봇 주변에 수렴해야 합니다

2. **맵 확인**:
   ```bash
   # 맵이 현재 환경과 맞는지 확인
   eog ~/ros2_ws/install/louvre_navigation/share/louvre_navigation/maps/louvre_map.png
   ```

3. **AMCL 파라미터 조정** (`louvre_navigation_params.yaml`):
   ```yaml
   amcl:
     min_particles: 200      # 파티클 수 증가
     max_particles: 2000
     update_min_d: 0.1       # 업데이트 임계값 감소
     update_min_a: 0.1
   ```

### 5. "Robot is stuck / Recovery failed"

**원인**: 로봇이 장애물에 갇혔거나 경로를 찾을 수 없음

**해결**:
1. **수동으로 로봇 이동**:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

2. **Recovery Behavior 확인**:
   - RViz에서 로봇이 회전하거나 후진하는지 확인
   - 충분한 공간이 없으면 Recovery도 실패

3. **Costmap 확인**:
   - RViz에서 빨간색 영역(장애물)이 과도하게 넓으면
   - `inflation_radius` 감소 (0.5로 줄이기)

### 6. "Robot drives too fast / aggressive"

**원인**: 속도 파라미터가 너무 높음

**해결** (`louvre_navigation_params.yaml` 수정):
```yaml
max_vel_x: 0.3              # 0.5 → 0.3으로 감소
max_vel_theta: 0.7          # 1.0 → 0.7로 감소

FollowPath:
  max_vel_x: 0.3
  min_speed_xy: 0.0         # 최소 속도 0으로 (부드럽게 정지)
```

### 7. "Oscillation / Wobbling"

**원인**: 경로 추종 컨트롤러가 불안정

**해결** (`louvre_navigation_params.yaml` 수정):
```yaml
FollowPath:
  path_distance_bias: 32.0      # 경로 추종 우선순위 증가
  goal_distance_bias: 24.0      # 목표 접근 우선순위
  xy_goal_tolerance: 0.15       # 목표 허용 오차 증가
  yaw_goal_tolerance: 0.25      # 각도 허용 오차 증가
```

---

## RViz 인터페이스 설명

### 주요 디스플레이

1. **Map** (회색):
   - SLAM 또는 Map Server에서 로드된 맵
   - 검정: 장애물
   - 흰색: 자유 공간
   - 회색: 미탐사 영역

2. **Global Costmap** (파란색/보라색):
   - 전체 경로 계획용 장애물 지도
   - 파란색: 안전 영역
   - 보라색/빨강: 장애물

3. **Local Costmap** (빨간색):
   - 지역 경로 계획용 장애물 지도
   - 실시간 업데이트
   - 로봇 주변 5m x 5m

4. **Global Path** (파란색 선):
   - Planner가 계산한 전체 경로
   - A* 또는 Dijkstra 알고리즘

5. **Local Path** (초록색 선):
   - Controller가 추종하는 지역 경로
   - 실시간으로 장애물 회피

6. **Particle Cloud** (초록색 화살표):
   - AMCL의 위치 추정 파티클
   - 수렴할수록 위치 확실

7. **LaserScan** (빨간색 점):
   - LiDAR 센서 데이터
   - 360도 스캔

### 주요 버튼

- **2D Pose Estimate**: 초기 위치 설정
- **Nav2 Goal**: 목표 지점 설정
- **Publish Point**: 특정 지점 퍼블리시
- **Measure**: 거리 측정

---

## 고급 설정

### 1. 다른 Planner 사용

**NavFn** (기본):
```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.5
  use_astar: false          # Dijkstra
```

**A*** (더 빠름):
```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.5
  use_astar: true           # A* 알고리즘
```

**Smac Planner** (부드러운 경로):
```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlannerHybrid"
  tolerance: 0.5
  downsample_costmap: false
  downsampling_factor: 1
```

### 2. 다른 Controller 사용

**TEB** (Time Elastic Band, 더 부드러움):
```yaml
FollowPath:
  plugin: "teb_local_planner::TebLocalPlannerROS"
  max_vel_x: 0.5
  max_vel_theta: 1.0
  # TEB은 별도 파라미터 많음
```

### 3. Behavior Tree 커스터마이징

Nav2는 Behavior Tree로 동작합니다:

```xml
<!-- custom_bt.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}"/>
      </RateController>
      <FollowPath path="{path}"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

---

## 참고 자료

- [Nav2 공식 문서](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Isaac Sim ROS2 Bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)

---

## 체크리스트

### SLAM 실행 전
- [ ] Isaac Sim 실행 및 씬 로드
- [ ] Play 버튼 클릭
- [ ] `/odom`, `/front_laser/scan`, `/rear_laser/scan` 토픽 확인
- [ ] TF 트리 확인 (`odom` → `base_link`)

### Nav2 실행 전
- [ ] 맵 파일 생성 완료 (`louvre_map.yaml`, `louvre_map.png`)
- [ ] 맵 파일이 올바른 경로에 있는지 확인
- [ ] Isaac Sim Play 버튼 클릭
- [ ] 토픽 및 TF 확인

### 네비게이션 중
- [ ] 2D Pose Estimate로 초기 위치 설정
- [ ] AMCL 파티클이 수렴하는지 확인
- [ ] Nav2 Goal 설정
- [ ] Global Path와 Local Path가 보이는지 확인
- [ ] 로봇이 경로를 따라 이동하는지 확인

---

**작성자**: AI Assistant  
**날짜**: 2026년 1월 6일  
**버전**: 1.0
