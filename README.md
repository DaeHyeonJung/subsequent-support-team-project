# Formation Flight 2D Simulator

편대 재편성 의사결정 지원 SW를 위한 1차 2D 시뮬레이션 환경입니다.

현재 버전은 표준 Python 라이브러리만 사용합니다.-test

- 2개 편대, 편대별 5대 UAV 생성
- 역할: 정찰 1대, 타격 2대, 기만 2대
- 점질량 기반 2D 운동
- 롤 명령 1차 지연 기반 pseudo dynamics
- 직진 비행 및 선택적 선회 명령
- 궤적 CSV와 SVG 시각화 출력

## 실행

PowerShell에서 프로젝트 폴더로 이동한 뒤 실행합니다.

```powershell
cd C:\Users\hoyad\Desktop\Formation_Flight
python formation_sim.py
```

실시간 화면을 보고 싶으면 아래 파일을 더블클릭합니다.

```text
실시간시뮬레이션실행.bat
```

또는 PowerShell에서 다음처럼 실행합니다.

```powershell
python run_realtime_sim.py
```

실시간 화면에서는 `Pause`, `Reset`, `UAV Speed`, `Tail Length`를 조절할 수 있습니다.

출력 파일:

- `outputs/trajectory.csv`: 시간별 UAV 상태
- `outputs/trajectory.svg`: 2D 궤적 시각화

선회 예시:

```powershell
python formation_sim.py --turn-demo --output-dir outputs/turn_demo
```

직진 시나리오와 선회 시나리오를 따로 저장하려면 다음처럼 실행합니다.

```powershell
python formation_sim.py --output-dir outputs/straight
python formation_sim.py --turn-demo --output-dir outputs/turn_demo
```

## 코드 구조

연구/학위논문/SCI 논문 확장을 고려해 CSCI, CSC, CSU 단위로 나누었습니다.

```text
src/
  CSCI_Simulation_Engine/
    CSC_Application/
      CSU_CLI.py
      CSU_SimulationRunner.py
    CSC_Command/
      CSU_RollCommand.py
    CSC_Configuration/
      CSU_SimConfig.py
    CSC_Dynamics/
      CSU_PointMassPseudoDynamics.py
    CSC_Models/
      CSU_UavState.py
    CSC_Output/
      CSU_CsvWriter.py
      CSU_SvgWriter.py
    CSC_Scenario/
      CSU_InitialScenario.py
    CSC_Interface/
      CSU_SimulationPort.py
    CSC_Visualization/
      CSU_RealtimeTkViewer.py
  CSCI_Reconfiguration_Decision/
    CSC_StateBus/
      CSU_TelemetryMessage.py
      CSU_StateBus.py
      CSU_OperationalState.py
    CSC_Availability/
      CSU_AvailabilityEvaluator.py
    CSC_RolePriority/
      CSU_RolePriority.py
```

`CSC_Interface/CSU_SimulationPort.py`는 나중에 Gazebo, Unreal, ROS2, AirSim 같은 외부 시뮬레이터와 연결하기 위한 포트입니다.
현재 실시간 화면은 `NullSimulationPort`를 사용하므로 외부로 보내지는 데이터는 없지만, 내부 시뮬레이션 상태는 `SimulationSnapshot` 형태로 묶을 수 있게 해두었습니다.

`CSCI_Reconfiguration_Decision/CSC_StateBus`는 기체별 상태 메시지를 모으는 내부 토픽 버스 역할을 합니다.
기체는 위치, 배터리, 역할, 편대 번호 같은 자기 상태만 발행하고, 기체 간 거리는 `StateBus`가 위치를 이용해 계산합니다.

## 동역학 모델

각 UAV는 일정 속도 `v`로 비행하는 점질량으로 표현합니다.

```text
x_dot = v cos(psi)
y_dot = v sin(psi)
psi_dot = g / v * tan(phi)
phi_dot = (phi_cmd - phi) / tau_phi
```

여기서 `phi_cmd`는 롤 명령, `phi`는 실제 롤각, `psi`는 heading입니다.
