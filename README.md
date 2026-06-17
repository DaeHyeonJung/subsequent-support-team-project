# Formation Flight Simulator

UAV 편대 재구성 의사결정 지원을 위한 Python 기반 시뮬레이션 프로젝트입니다.

현재 코드는 기본 배치 시뮬레이션, 실시간 Tk 시각화, 편대 재구성 슬롯 할당, 3D 유도/충돌회피, LQR 기반 슬롯 추종 제어, Aerosonde 기준 공력 파라미터를 포함합니다.

## 주요 기능

- 2개 편대, 편대별 5대 UAV 초기 시나리오 생성
- 정찰, 타격, 기만 역할 기반 우선순위 평가
- 배터리 소모 및 랜덤 kill event 반영
- Fair Hungarian 기반 슬롯 할당
- wedge, line, column, staggered column 편대 형상 선택
- 3D potential field 기반 충돌회피
- lateral slot LQR 및 vertical LQR 기반 편대 슬롯 추종
- Aerosonde 기준 질량, 날개 면적, 공력 계수, trim 계수 설정
- RK4 기반 pseudo dynamics 적분
- CSV 및 SVG 출력
- Tk 기반 실시간 2D/3D 시각화

## 요구 사항

Python 3.10 이상을 권장합니다.

LQR 컨트롤러는 `numpy`, `scipy`를 사용합니다.

```powershell
pip install numpy scipy
```

일부 테스트/시각화 실험 파일은 `matplotlib`을 사용할 수 있습니다.

```powershell
pip install matplotlib
```

## 실행 방법

기본 배치 시뮬레이션:

```powershell
python formation_sim.py
```

선회 명령 예시:

```powershell
python formation_sim.py --turn-demo --output-dir outputs\turn_demo
```

실시간 시뮬레이터:

```powershell
python run_realtime_sim.py
```

또는 Windows 배치 파일을 사용할 수 있습니다.

```powershell
.\Sim_run.bat
```

## 배치 시뮬레이션 옵션

`formation_sim.py`는 다음 옵션을 지원합니다.

```powershell
python formation_sim.py --duration 80 --dt 0.1 --speed 15 --output-dir outputs --turn-demo
```

- `--duration`: 시뮬레이션 시간, 초 단위
- `--dt`: 적분 시간 간격, 초 단위
- `--speed`: 초기 UAV 속도, m/s
- `--output-dir`: 출력 폴더
- `--turn-demo`: 단순 roll command 기반 선회 기동 적용

## 출력 파일

배치 실행 시 지정한 출력 폴더에 다음 파일이 생성됩니다.

- `trajectory.csv`: 시간별 UAV 상태
- `trajectory.svg`: 궤적 시각화

CSV에는 위치, 고도, 속도, heading, flight path angle, roll, roll rate, 종방향/수직 가속도, 배터리 및 상태 정보가 포함됩니다.

## 실시간 시뮬레이터

`run_realtime_sim.py`는 `CSU_RealtimeTkViewer`를 실행합니다.

실시간 화면에서는 다음 기능을 사용할 수 있습니다.

- Pause/Reset
- 속도 조절
- tail length 조절
- follow camera
- 편대 형상 선택
- 역할 우선순위 조정
- kill event 이후 재구성 확인
- 2D top view, 3D preview, side view 기반 상태 확인

실시간 재구성 경로에서는 `compute_virtual_structure_tracking_command()`가 `roll_cmd_rad`, `desired_flight_path_rad`, `speed_cmd_mps`를 직접 반환합니다. 이 명령은 lateral/vertical LQR 컨트롤러를 거쳐 `step_uav()`에 전달됩니다.

## 동역학 모델

동역학은 `src/CSCI_Simulation_Engine/CSC_Dynamics/CSU_PointMassPseudoDynamics.py`에 있습니다.

상태는 다음 변수를 중심으로 적분합니다.

```text
x = [x, y, z, V, psi, gamma, phi, phi_dot]^T
```

- `x, y, z`: 3D 위치
- `V`: 속도
- `psi`: heading
- `gamma`: flight path angle
- `phi`: roll angle
- `phi_dot`: roll rate

적분은 RK4를 사용합니다. Roll inner-loop는 `RollPDController`, 속도 명령은 `SpeedController`, flight path angle은 2차 제한 모델을 통해 갱신됩니다.

중력, 질량, 날개 면적, 공력 계수는 `SimConfig`의 Aerosonde 기준 설정을 따릅니다. 대표 계수는 trim 계산 결과인 `lift_coefficient`, `drag_coefficient`, `thrust_coefficient`로 pseudo dynamics에 사용됩니다.

## 제어 및 유도

주요 유도/제어 모듈은 `src/CSCI_Guidance_Control` 아래에 있습니다.

- `CSC_Controller/CSU_LateralSlotLQRController.py`: lateral slot tracking용 roll command LQR
- `CSC_Controller/CSU_VerticalLQRController.py`: altitude tracking용 flight path command LQR
- `CSC_Controller/CSU_RollPDController.py`: roll inner-loop PD
- `CSC_Controller/CSU_SpeedController.py`: 속도 제어
- `CSC_Controller/CSU_HeadingController.py`: heading command를 roll command로 변환하는 보조 컨트롤러
- `CSC_Guidance/CSU_BasicLOS3DGuidance.py`: 3D LOS 유도
- `CSC_Guidance/CSU_SlotReferenceGenerator.py`: 슬롯 기준점 smoothing
- `CSC_CollisionAvoidance/CSU_PotentialField3DAvoidance.py`: 3D potential field 충돌회피

## 편대 재구성

재구성 의사결정 모듈은 `src/CSCI_Reconfiguration_Decision` 아래에 있습니다.

- `CSC_FormationManagement/CSU_FormationSlots.py`: 편대 형상별 슬롯 정의
- `CSC_FormationManagement/CSU_SlotAllocation.py`: 슬롯 할당
- `CSC_FormationManagement/CSU_FairHungarian.py`: Fair Hungarian 할당
- `CSC_FormationManagement/CSU_FormationManager.py`: 형상 선택과 슬롯 목표 좌표 생성
- `CSC_FormationManagement/CSU_ReconfigurationEvaluator.py`: 재구성 계획 평가
- `CSC_RolePriority`: 역할 우선순위 평가
- `CSC_Availability`: 기체 가용성 평가
- `CSC_StateBus`: UAV 상태 메시지 수집 및 거리 계산

## 코드 구조

```text
src/
  CSCI_Guidance_Control/
    CSC_CollisionAvoidance/
    CSC_Controller/
    CSC_Guidance/
  CSCI_Reconfiguration_Decision/
    CSC_Availability/
    CSC_FormationManagement/
    CSC_Reconfiguration/
    CSC_RolePriority/
    CSC_StateBus/
  CSCI_Simulation_Engine/
    CSC_Application/
    CSC_Battery/
    CSC_Command/
    CSC_Configuration/
    CSC_Dynamics/
    CSC_Failure/
    CSC_Interface/
    CSC_Models/
    CSC_Output/
    CSC_Scenario/
    CSC_Visualization/
```

루트 실행 파일:

- `formation_sim.py`: 배치 시뮬레이션 실행
- `run_realtime_sim.py`: 실시간 Tk 시뮬레이터 실행
- `Sim_run.bat`: Windows 배치 실행 파일
- `test_visualize_algorithms.py`: 알고리즘 시각화/스모크 테스트용 파일

