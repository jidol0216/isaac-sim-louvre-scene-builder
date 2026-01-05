# Louvre Scene Builder - Extension 최소 파일

이 디렉토리는 **Extension 실행에 필요한 파일만** 포함합니다.  
**어느 경로에 복사해도 바로 실행 가능합니다!** ✨

## 📂 디렉토리 구조

```
louvre_extension_minimal/
├── exts/
│   └── louvre.scene.builder/      # Extension 코드
│       ├── extension.toml         # Extension 설정
│       └── louvre/scene/builder/
│           └── extension.py       # 메인 로직 (상대 경로 사용)
├── assets/
│   ├── robots/
│   │   └── RidgebackFranka/       # 로봇 USD (ActionGraph 포함)
│   └── wheel/                     # Mecanum wheel USD
├── scenes/
│   └── lubre.glb                  # Louvre 환경 (텍스처 포함)
├── isaac-sim-lou.sh               # Isaac Sim 실행 스크립트
└── README.md                      # 이 파일
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

또는 직접 실행:

```bash
cd /path/to/isaac-sim
./isaac-sim.sh --ext-folder /path/to/louvre_extension_minimal/exts
```

### 3. Extension 활성화

- **Window** → **Extensions**
- 검색: `louvre`
- **ENABLED** 토글

### 4. 씬 빌드

- **Build Louvre Scene** 버튼 클릭
- **Play** 버튼

## 📦 포함된 파일 설명

### Extension 코드
- `extension.py`: 메인 로직 (씬 구성, 저장/불러오기)
- `extension.toml`: Extension 설정

### Assets
- **로봇**: RidgebackFranka (ActionGraph 포함)
- **휠**: Mecanum wheel (마찰력 설정 포함)
- **환경**: Louvre 박물관 GLB (텍스처 임베드)

### 자동 다운로드 (인터넷 필요)
- RealSense D455 카메라
- SICK TiM781 LiDAR

## 📏 파일 크기

```bash
du -sh louvre_extension_minimal/
```

약 600MB (Louvre GLB 포함)

## 🔧 이식성 (Portability)

**상대 경로를 사용하므로 어느 경로에 복사해도 동작합니다!**

- `extension.py`: `Path(__file__)` 기준 상대 경로 사용
- `isaac-sim-lou.sh`: `$SCRIPT_DIR` 기준 상대 경로 사용
- `generate_map.py`: 스크립트 위치 기준 상대 경로 사용

## 💡 팁

- 이 디렉토리만 복사하면 다른 컴퓨터에서도 즉시 실행 가능
- Isaac Sim과 인터넷 연결만 있으면 OK
- 폴더 이름 변경 가능 (경로 수정 불필요)
- 다른 사용자명에서도 그대로 동작
