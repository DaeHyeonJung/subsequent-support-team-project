# 05/14 수업발표 내용 정리

## 작업 브랜치

- 작업 브랜치: `feature/DaeHyeon`
- 목적: 2D 편대비행 시뮬레이션에서 UAV 상태정보를 실시간으로 기록하고, 무작위 격추 이벤트를 반영할 수 있도록 기능 확장

## 전체 방향

최종 목표는 시뮬레이션 실행 중 UAV들의 상태정보를 실시간으로 받아오고, 해당 상태정보를 기반으로 사용 가능 여부와 편대 재편성 판단을 수행하는 것이다.

현재 구조에서는 실시간 판단용 데이터 흐름과 결과 분석용 파일 저장을 분리해서 사용한다.

- 실시간 판단용: `StateBus`
- 분석 및 기록용: `outputs/trajectory.csv`, `outputs/trajectory.svg`

CSV 파일은 나중에 결과를 분석하기 위한 로그이고, 실시간 재편성 판단은 CSV를 다시 읽는 방식이 아니라 `StateBus`에 저장된 최신 UAV 상태를 사용하는 방식으로 진행하는 것이 적절하다.

## outputs 저장 방식 정리

기존에는 `formation_sim.py`를 실행할 때만 `outputs/` 폴더가 생성되었다.

변경 후에는 `Sim_run.bat`을 실행해 실시간 시뮬레이션을 돌려도 자동으로 결과 폴더가 생성된다.

생성 예시:

```text
outputs/
  realtime_YYYYMMDD_HHMMSS_microsecond/
    trajectory.csv
    trajectory.svg
```

동작 방식:

- 실시간 시뮬레이션 시작 시 새 output session 생성
- 매 tick마다 `trajectory.csv`에 UAV 상태 기록
- 창을 닫거나 Reset을 누르면 현재 궤적을 `trajectory.svg`로 저장
- Reset 시 기존 세션을 마무리하고 새 세션을 다시 생성

## .gitignore 수정

실행 결과 파일이 GitHub에 올라가지 않도록 `.gitignore`에 아래 항목을 추가했다.

```gitignore
outputs/
```

따라서 시뮬레이션 실행으로 생성되는 CSV/SVG 결과물은 Git 추적 대상에서 제외된다.

## 배터리 기록 로직 개선

기존 문제:

- 실시간 화면에서는 배터리 감소가 반영되고 있었음
- 하지만 `formation_sim.py`로 생성한 CSV에는 시간별 배터리 값이 제대로 기록되지 않았음
- `UavState.history`가 위치와 자세만 저장하고 있었고, CSV 작성 시 최종 `battery_pct` 값 하나를 모든 행에 쓰는 구조였음

개선 내용:

- `UavState.history`에 시간별 `battery_pct` 추가
- 배치 시뮬레이션 루프에도 `BatteryModel` 연결
- CSV 작성 시 각 시간의 배터리 값을 기록하도록 수정
- SVG 작성 코드와 실시간 꼬리 그리기 코드도 변경된 history 구조에 맞게 수정

관련 파일:

```text
src/CSCI_Simulation_Engine/CSC_Models/CSU_UavState.py
src/CSCI_Simulation_Engine/CSC_Application/CSU_SimulationRunner.py
src/CSCI_Simulation_Engine/CSC_Output/CSU_CsvWriter.py
src/CSCI_Simulation_Engine/CSC_Output/CSU_SvgWriter.py
src/CSCI_Simulation_Engine/CSC_Visualization/CSU_RealtimeTkViewer.py
```

## StateBus 역할 정리

`StateBus`는 실시간으로 들어오는 UAV 상태를 모아두는 내부 상태 버스 역할을 한다.

위치:

```text
src/CSCI_Reconfiguration_Decision/CSC_StateBus/CSU_StateBus.py
```

현재 역할:

- UAV별 최신 telemetry 저장
- `SimulationSnapshot`을 `UavTelemetryMessage`로 변환
- 현재 최신 UAV 상태 목록 제공
- `AvailabilityEvaluator`를 통해 사용 가능한 UAV 목록 제공
- UAV 간 거리 계산 제공

전체 흐름:

```text
SimulationSnapshot
  -> StateBus.update_from_simulation_snapshot(...)
  -> StateBus.latest_telemetry()
  -> StateBus.operational_states(...)
  -> StateBus.available_uavs(...)
```

## 무작위 격추 이벤트 로직 추가

새로운 CSC로 `CSC_Failure`를 추가했다.

위치:

```text
src/CSCI_Simulation_Engine/CSC_Failure/
  __init__.py
  CSU_KillEventConfig.py
  CSU_RandomKillEventModel.py
```

### 격추 이벤트 요구사항

- 시뮬레이션 시작 후 5초부터 격추 이벤트 시작
- 0.5초 간격으로 총 3대 격추
- 이벤트 시간:

```text
t = 5.0초
t = 5.5초
t = 6.0초
```

- 두 편대 중 하나는 2대 격추
- 나머지 편대는 1대 격추
- 어느 편대가 2대 격추될지는 매 시뮬레이션마다 무작위
- 정찰기 역할인 `recon`은 총 2대뿐이므로, 한 대가 격추되면 남은 정찰기 한 대는 격추 후보에서 제외
- 즉, 정찰기 2대가 모두 격추되는 상황은 발생하지 않도록 제한

### 격추된 UAV 상태

격추된 UAV는 다음 상태로 변경된다.

```text
available = False
link_ok = False
vehicle_health = "KILLED"
payload_ok = False
```

`AvailabilityEvaluator`에서는 격추된 UAV가 `VEHICLE_KILLED` 사유로 사용 불가 판정되도록 판단 순서를 조정했다.

관련 파일:

```text
src/CSCI_Reconfiguration_Decision/CSC_Availability/CSU_AvailabilityEvaluator.py
```

## 실시간 시뮬레이션에 격추 이벤트 연결

격추 이벤트 모델을 실시간 시뮬레이션 루프에 연결했다.

위치:

```text
src/CSCI_Simulation_Engine/CSC_Visualization/CSU_RealtimeTkViewer.py
```

현재 동작:

- `RandomKillEventModel`을 실시간 뷰어에서 생성
- 매 tick마다 현재 시간이 격추 이벤트 시간에 도달했는지 확인
- 조건에 맞는 UAV를 무작위로 격추
- 격추된 UAV는 더 이상 이동하지 않음
- 격추된 UAV도 CSV에는 계속 상태가 기록됨
- StateBus에는 격추 이후 상태가 반영됨

흐름:

```text
advance_simulation()
  1. 살아있는 UAV 이동
  2. 살아있는 UAV 배터리 감소
  3. 격추 이벤트 적용
  4. SimulationSnapshot 생성
  5. StateBus 업데이트
  6. CSV 기록
```

## 화면 표시 개선

실시간 화면에서 격추된 기체를 확인할 수 있도록 표시를 추가했다.

화면 중앙:

- 격추된 UAV 위치에 `X` 마커 표시
- `F?-U? KILLED` 라벨 표시
- 격추된 UAV는 마지막 위치에 멈춰 있음

HUD:

- 격추된 UAV 목록을 `Kill events`로 표시
- 사용 가능한 UAV 수 표시
- 사용 가능한 정찰기 수 표시

오른쪽 상태 패널:

- 기존 `UAV Battery` 패널을 `UAV Status` 패널로 변경
- 컬럼 구성:

```text
Form | Role | Battery | Status
```

- 정상 UAV는 `Status = OK`
- 격추된 UAV는 `Status = KILLED`
- 격추된 UAV의 `Role` 칸 배경색은 검정색으로 표시

## CSV에 기록되는 주요 상태

실시간 CSV에는 다음 정보가 기록된다.

```text
time_s
uav_id
formation_id
role
available
availability_reason
battery_pct
link_ok
vehicle_health
payload_ok
x_m
y_m
heading_deg
roll_deg
```

격추된 UAV의 예시 상태:

```text
available = 0
availability_reason = VEHICLE_KILLED
vehicle_health = KILLED
payload_ok = 0
link_ok = 0
```

## 검증 내용

문법 검증:

```powershell
python -m compileall src formation_sim.py run_realtime_sim.py
```

검증 결과:

- 컴파일 오류 없음
- 격추 이벤트가 총 3대 발생하는 것 확인
- 한 편대 2대, 다른 편대 1대 격추 조건 확인
- 정찰기 최소 1대 생존 조건 확인
- 격추된 UAV가 `VEHICLE_KILLED`로 사용 불가 판정되는 것 확인

## 현재 남은 작업 후보

- 격추 이벤트 발생 내역을 별도 event log 컬럼 또는 파일로 분리
- 격추 이후 편대 재편성 알고리즘 추가
- 재편성 결과를 화면과 CSV에 기록
- `StateBus`에서 재편성 판단에 필요한 데이터 API 정리
- README 삭제/변경 상태 확인 후 복구 또는 재작성
