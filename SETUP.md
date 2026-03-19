# DefenseKit - 프로젝트 설정 가이드

## 프로젝트 소개

DefenseKit은 C++17 기반의 국방/방산 알고리즘 라이브러리입니다. 탄도 계산(Ballistics), 신호 처리(Signal Processing), 레이더 표적 추적(Radar Target Tracking), 관성 항법(INS Navigation) 등의 핵심 알고리즘을 모듈화하여 제공합니다.

- **기술 스택**: C++17, CMake 3.16+
- **주요 모듈**: ballistics, signal_processing (FFT, Kalman Filter), radar, navigation

---

## 필요한 API 키 / 환경변수

DefenseKit은 순수 C++ 라이브러리로, 별도의 API 키나 외부 서비스 연동이 필요하지 않습니다.

---

## GitHub Secrets 설정 방법

GitHub Actions를 통해 빌드 및 테스트를 자동화하는 경우, 별도로 등록할 Secrets는 없습니다.

CI/CD 워크플로우 예시 (`.github/workflows/build.yml`):

```yaml
name: Build and Test
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Configure CMake
        run: cmake -B build -DCMAKE_BUILD_TYPE=Release
      - name: Build
        run: cmake --build build
      - name: Test
        run: ctest --test-dir build
```

---

## 로컬 개발 환경 설정

### 사전 요구사항

- CMake 3.16 이상
- C++17 지원 컴파일러 (GCC 7+, Clang 5+, MSVC 2017+)

### 1. 저장소 클론

```bash
git clone https://github.com/sconoscituo/DefenseKit.git
cd DefenseKit
```

### 2. 빌드 디렉토리 생성 및 CMake 구성

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```

### 3. 빌드

```bash
cmake --build . --parallel
```

---

## 실행 방법

### 예제 실행

빌드 완료 후 `build/` 디렉토리에서 예제 바이너리를 실행합니다.

```bash
# 탄도 계산 데모
./ballistics_demo
```

### 테스트 실행

```bash
cd build
ctest --output-on-failure
```

### 라이브러리로 사용 (CMake 연동)

```cmake
add_subdirectory(DefenseKit)
target_link_libraries(your_target defensekit_lib)
```
