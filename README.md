# Louvre Scene Builder - Extension 최소 파일

이 디렉토리는 **Extension 실행에 필요한 파일만** 포함합니다.

## 📂 디렉토리 구조

```
louvre_extension_minimal/
├── louvre.scene.builder/          # Extension 코드
│   ├── extension.toml             # Extension 설정
│   └── louvre/scene/builder/
│       └── extension.py           # 메인 로직
├── assets/
│   ├── robots/
│   │   └── RidgebackFranka/       # 로봇 USD (ActionGraph 포함)
│   └── wheel/                     # Mecanum wheel USD
├── scenes/
│   └── lubre.glb                  # Louvre 환경 (텍스처 포함)
└── README.md                      # 이 파일
```

## 🚀 실행 방법

### 1. Extension 폴더 구조 맞추기

현재 구조를 Extension 표준 구조로 변경:

```bash
cd /home/rokey/env_set/louvre_extension_minimal
mkdir -p exts
mv louvre.scene.builder exts/
```

### 2. Isaac Sim 실행

```bash
cd /path/to/isaac-sim
./isaac-sim.sh --ext-folder /home/rokey/louvre_extension_minimal/exts
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

## 🔧 경로 수정

다른 위치로 이동 시 `extension.py` 수정:

```python
ENV_SET_DIR = Path("/home/rokey/louvre_extension_minimal")
```

## 💡 팁

- 이 디렉토리만 복사하면 다른 컴퓨터에서도 실행 가능
- Isaac Sim과 인터넷 연결만 있으면 OK
- 원본 `/home/rokey/env_set/`와 독립적으로 작동
- 현재 위치: `/home/rokey/louvre_extension_minimal/`
