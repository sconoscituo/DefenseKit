# DefenseKit

C++17 방산/임베디드 포트폴리오 프로젝트. 한화시스템, LIG넥스원, 현대로템 등 방위산업체 지원을 위한 C++ 알고리즘/시스템 구현 모음.

## 구현 모듈

### 탄도학 (Ballistics)
- **Runge-Kutta 4차 수치 적분**으로 탄도 궤적 계산
- 중력, 공기저항, Coriolis 효과 포함
- 국제 표준 대기 모델 (ISA) 적용
- 마하수 기반 항력 계수 가변 적용
- 역탄도 계산 (목표 거리 → 최적 앙각 이분법 탐색)

### 신호처리 (Signal Processing)
- **Cooley-Tukey FFT** (반복적 구현, 캐시 효율 최적화)
- 비트 역순 치환 전처리
- 파워 스펙트럼 밀도 계산 (dB)
- 레이더 도플러 피크 주파수 탐색

### 칼만 필터 (Kalman Filter)
- **선형 칼만 필터** (상태: [x, y, vx, vy])
- 예측 단계: 상태 전이 행렬 기반 등속 운동 모델
- 업데이트 단계: 레이더 위치 측정값으로 보정
- 오차 공분산 전파 및 갱신

### 레이더 추적 (Radar Tracking)
- **다중 표적 추적기** (MHT 단순화)
- 게이팅 기반 측정값-트랙 연관
- 표적 확정/소멸 로직 (연속 탐지/미탐지 횟수 기반)
- 칼만 필터 내장으로 노이즈 필터링

### 관성항법 (INS)
- **Euler angle 기반 자세 추정** (ZYX 순서)
- Body → NED 좌표계 DCM 변환
- IMU 데이터 (가속도계 + 자이로) 적분
- 센서 바이어스 교정

### 행렬 연산 (Matrix)
- 고정 크기 행렬 템플릿 (Eigen 없이 직접 구현)
- 행렬 덧셈, 뺄셈, 곱셈, 전치
- 2x2, 4x4 역행렬 (칼만 게인 계산용)

## 수학적 배경

### 탄도 운동 방정식
```
m * a = F_gravity + F_drag + F_coriolis

F_drag = -0.5 * ρ(h) * v² * Cd(Ma) * A * v̂
F_coriolis = -2m * (Ω × v)
```

### 칼만 필터 알고리즘
```
예측: x̂⁻ = F·x̂,  P⁻ = F·P·Fᵀ + Q
업데이트: K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
          x̂ = x̂⁻ + K·(z - H·x̂⁻)
          P = (I - K·H)·P⁻
```

### FFT Cooley-Tukey 버터플라이
```
X[k] = Σ x[n] · e^(-j2πkn/N)
W_N^k = e^(-j2πk/N)  (회전 인자)
```

## 빌드 방법

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel

# 테스트 실행
ctest --output-on-failure

# 데모 실행
./ballistics_demo
./radar_demo
```

### 요구 사항
- CMake 3.16+
- GCC 9+ / Clang 10+ / MSVC 2019+ (C++17 지원)
- 인터넷 연결 (GoogleTest 자동 다운로드)

## 지원 가능 방산기업

| 기업 | 주요 사업 | 관련 모듈 |
|------|-----------|-----------|
| **한화시스템** | 전자전, 레이더, C4I | FFT, 칼만필터, 레이더추적 |
| **LIG넥스원** | 유도무기, 탐색기, 항법 | 칼만필터, INS, 탄도학 |
| **현대로템** | 전차, 자주포 사격통제 | 탄도학, 행렬연산 |
| **한국항공우주(KAI)** | 항공기, 헬기 항법 | INS, 칼만필터 |
| **풍산** | 탄약, 포탄 설계 | 탄도학, 대기모델 |
| **LG유플러스 방산** | 군 통신 시스템 | 신호처리, FFT |

## 디렉토리 구조

```
DefenseKit/
├── src/
│   ├── ballistics/      # 탄도학 시뮬레이션
│   ├── signal_processing/ # FFT + 칼만필터
│   ├── radar/           # 다중 표적 추적
│   ├── navigation/      # 관성항법시스템
│   └── utils/           # 행렬연산, 물리상수
├── tests/               # GoogleTest 단위 테스트
├── examples/            # 실행 가능한 데모
└── docs/                # 방산 취업 가이드
```
