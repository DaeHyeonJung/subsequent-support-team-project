import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment

# 프로젝트 루트 경로를 sys.path에 추가하여 모듈을 원활하게 임포트합니다.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.CSCI_Reconfiguration_Decision.CSC_FormationManagement.CSU_FairHungarian import fair_hungarian_assignment
from src.CSCI_Guidance_Control.CSU_WaypointGuidance import WaypointGuidance

def run_visual_test(num_points=7):
    """
    10대의 기체 중 3대가 손실(Killed)되어, 
    기존 2개의 편대(잔여 7대)가 1개의 새로운 편대로 재구성되는 시나리오입니다.
    """
    # 1. 시나리오 기반 무작위 군집 생성
    np.random.seed(42)  
    
    uavs = np.zeros((num_points, 2))
    slots = np.zeros((num_points, 2))
    
    # 기존 편대 1 (4대 생존): 좌측에 작은 Wedge(V자) 배치
    uavs[0:4] = [[25, 40], [15, 30], [35, 30], [5, 20]]
    # 기존 편대 2 (3대 생존): 우측에 작은 Wedge(V자) 배치
    uavs[4:7] = [[75, 40], [65, 30], [85, 30]]
    
    # 새로운 단일 편대 슬롯 (7개): 기체들보다 완전히 앞쪽(전방)에 지그재그 종렬 대형 형성
    slots[0] = [50, 120]  # 리더 (선봉)
    slots[1] = [55, 110]  # 우측 1번
    slots[2] = [45, 100]  # 좌측 1번
    slots[3] = [55, 90]   # 우측 2번
    slots[4] = [45, 80]   # 좌측 2번
    slots[5] = [55, 70]   # 우측 3번
    slots[6] = [45, 60]   # 좌측 3번

    # 2. 이동 거리(Cost) 행렬 계산
    cost_matrix = np.zeros((num_points, num_points))
    for i in range(num_points):
        for j in range(num_points):
            cost_matrix[i, j] = np.linalg.norm(uavs[i] - slots[j])

    # 3. 표준 헝가리안 알고리즘 (총 거리 최소화)
    std_row, std_col = linear_sum_assignment(cost_matrix)
    std_costs = cost_matrix[std_row, std_col]
    std_max_idx = np.argmax(std_costs)

    # 4. Fair 헝가리안 알고리즘 (최대 거리 최소화)
    fair_col = fair_hungarian_assignment(cost_matrix)
    fair_costs = cost_matrix[np.arange(num_points), fair_col]
    fair_max_idx = np.argmax(fair_costs)

    # 5. 궤적 시뮬레이션 (Guidance Control 적용)
    guidance = WaypointGuidance(max_speed=15.0, avoidance_radius=8.0, repulsive_gain=20.0)
    dt = 0.1
    max_steps = 500
    
    def simulate_trajectories(assignment):
        pos = uavs.copy()
        history = [pos.copy()]
        for _ in range(max_steps):
            vels = np.zeros_like(pos)
            reached_count = 0
            for i in range(num_points):
                target = slots[assignment[i]]
                if np.linalg.norm(target - pos[i]) < 0.5:
                    reached_count += 1
                    
                neighbors = [pos[j] for j in range(num_points) if j != i]
                vels[i] = guidance.compute_velocity_command(pos[i], target, neighbors)
                
            pos = pos + vels * dt
            history.append(pos.copy())
            if reached_count == num_points:
                break
        return np.array(history)

    std_history = simulate_trajectories(std_col)
    fair_history = simulate_trajectories(fair_col)

    # 6. 시각화 (Matplotlib)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle('Hungarian Algorithms Comparison', fontsize=16, fontweight='bold')

    # --- Plot 1: 표준 헝가리안 ---
    ax1 = axes[0]
    ax1.scatter(uavs[:, 0], uavs[:, 1], c='blue', label='UAV (Start)', marker='o', s=50)
    ax1.scatter(slots[:, 0], slots[:, 1], c='green', label='Slot (Target)', marker='X', s=70)
    
    for i in range(num_points):
        j = std_col[i]
        is_bottleneck = (i == std_max_idx)
        color = 'red' if is_bottleneck else 'gray'
        alpha = 1.0 if is_bottleneck else 0.4
        linewidth = 3.0 if is_bottleneck else 1.0
        zorder = 5 if is_bottleneck else 1
        ax1.plot(std_history[:, i, 0], std_history[:, i, 1], 
                 c=color, linewidth=linewidth, alpha=alpha, zorder=zorder)
                 
    ax1.set_title(f"Standard (Min Sum)\nTotal Distance: {std_costs.sum():.1f}m | Max Distance: {std_costs.max():.1f}m")
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.5)

    # --- Plot 2: Fair 헝가리안 ---
    ax2 = axes[1]
    ax2.scatter(uavs[:, 0], uavs[:, 1], c='blue', label='UAV (Start)', marker='o', s=50)
    ax2.scatter(slots[:, 0], slots[:, 1], c='green', label='Slot (Target)', marker='X', s=70)
    
    for i in range(num_points):
        j = fair_col[i]
        is_bottleneck = (i == fair_max_idx)
        color = 'red' if is_bottleneck else 'gray'
        alpha = 1.0 if is_bottleneck else 0.4
        linewidth = 3.0 if is_bottleneck else 1.0
        zorder = 5 if is_bottleneck else 1
        ax2.plot(fair_history[:, i, 0], fair_history[:, i, 1], 
                 c=color, linewidth=linewidth, alpha=alpha, zorder=zorder)
                 
    ax2.set_title(f"Fair (Minimax Bottleneck)\nTotal Distance: {fair_costs.sum():.1f}m | Max Distance: {fair_costs.max():.1f}m")
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_visual_test(num_points=7)