---
name: Encapsulation Refactor
description: 하드웨어 의존성을 HAL(하드웨어 추상화 계층)로 분리하기 위한 규칙
---

# 캡슐화 리팩토링 규칙 (Encapsulation Refactoring Rules)

## 목표
하드웨어 의존성(예: `plib.h` 및 레지스터 직접 접근)을 펌웨어 애플리케이션 로직으로부터 완벽하게 격리합니다.

## 핵심 규칙
1. **앱 계층에서 plib.h 사용 금지**:
   - `src` 디렉토리 내의 파일들은 절대 `<plib.h>`를 직접 `#include` 해서는 안 됩니다.
   - 단, `hal_lib/mlx813xx` (하드웨어 구현 폴더) 내부는 예외입니다.

2. **HAL 인터페이스 사용 준수**:
   - 모든 하드웨어 제어(GPIO, PWM, ADC 등)는 반드시 미리 정의된 `HAL_xxx()` 인터페이스 함수를 통해서만 수행해야 합니다.
   - 코드 내에서 `REG_...` 와 같은 레지스터 직접 접근 매크로 사용을 금지합니다.

3. **파일 구조 및 네이밍 규칙**:
   - **인터페이스 정의 (공통)**: `hal_lib/interface/hal_xxx.h`
   - **구현체 (MLX813xx 전용)**: `hal_lib/mlx813xx/hal_xxx.c`
