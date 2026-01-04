# Louvre Scene Builder Extension

Isaac Sim에서 Louvre 박물관 씬을 자동으로 조립하는 Extension입니다.

## 장점
- **텍스처 유지**: 원본 USD를 reference로 불러오므로 텍스처 경로가 깨지지 않음
- **ActionGraph 유지**: 로봇 USD를 reference로 불러오므로 ActionGraph 보존
- **저장 불필요**: 매번 원본에서 조립하므로 USD 저장할 필요 없음

## 설치 방법

### 방법 1: Isaac Sim 설정에 Extension 경로 추가
```bash
# Isaac Sim 실행 시 extension 경로 추가
cd /home/rokey/isaacsim
./isaac-sim.sh --ext-folder /home/rokey/env_set/exts
```

### 방법 2: 영구적으로 Extension 경로 추가
Isaac Sim 실행 후:
1. `Window` → `Extensions`
2. 상단의 ⚙️ (설정) 클릭
3. `Extension Search Paths`에 `/home/rokey/env_set/exts` 추가
4. `louvre.scene.builder` 검색 후 활성화

## 사용 방법
1. Isaac Sim 실행
2. Extension 활성화 (위 설치 방법 참조)
3. "Louvre Scene Builder" 창에서 "🏛️ Build Louvre Scene" 클릭
4. Space 키로 시뮬레이션 시작

## 파일 구조
```
exts/
└── louvre.scene.builder/
    ├── extension.toml          # Extension 설정
    └── louvre/scene/builder/
        ├── __init__.py
        └── extension.py        # 메인 로직
```

## 참조하는 원본 파일들
- Louvre: `/home/rokey/env_set/scenes/output/origin_lubre.usd`
- Robot: `/home/rokey/env_set/assets/robots/RidgebackFranka/ridgeback_franka.usd`
- Mecanum: `/home/rokey/env_set/assets/wheel/basic_four_mecanum_robot_after_add_friction_for_rollers_2.usd`

이 파일들의 텍스처/ActionGraph가 원본 그대로 유지됩니다!
