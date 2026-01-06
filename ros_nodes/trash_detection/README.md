# Trash Detection ROS2 Package

쓰레기 감지를 위한 YOLO 기반 ROS2 노드입니다.

## 설치 방법

### 1. 패키지 복사
```bash
# humble_ws/src에 패키지 복사
cp -r trash_detection ~/IsaacSim-ros_workspaces/humble_ws/src/
```

### 2. Python 의존성 설치
```bash
pip install -r requirements.txt
```

### 3. 빌드
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws
colcon build --packages-select trash_detection
source install/setup.bash
```

## 실행 방법
```bash
ros2 run trash_detection test_dect
```

## 필요 토픽
- **Subscribe**: `/rgb` (sensor_msgs/Image) - RGB 이미지 입력
- **Publish**: `/trash_detections` (sensor_msgs/Image) - 탐지 결과 이미지

## 파일 구조
```
trash_detection/
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
├── README.md
├── resource/
│   └── trash_detection
├── trash_detection/
│   ├── __init__.py
│   ├── test_dect.py      # 메인 노드
│   ├── trash.pt          # YOLO 모델 (50MB)
│   └── trash1.pt         # YOLO 모델 (84MB) - 최신버전
└── test/
```

## 모델 파일
- `trash.pt` - 기본 모델 (50MB)
- `trash1.pt` - 개선된 모델 (84MB) ← 현재 사용 중

⚠️ **주의**: `.venv` 폴더는 포함하지 마세요! (7GB 넘음)
