import numpy as np

class WaypointGuidance:
    def __init__(self, max_speed=5.0, avoidance_radius=3.0, repulsive_gain=2.0):
        """
        UAV의 목표점 유도 및 충돌 회피를 위한 제어기
        :param max_speed: 기체의 최대 이동 속도
        :param avoidance_radius: 회피 기동을 시작할 주변 기체와의 최소 반경
        :param repulsive_gain: 반발력(밀어내는 힘)의 강도
        """
        self.max_speed = max_speed
        self.avoidance_radius = avoidance_radius
        self.repulsive_gain = repulsive_gain

    def compute_velocity_command(self, current_pos, target_pos, neighbor_positions=None):
        """
        현재 위치에서 목표 위치로 향하는 2D 속도 벡터를 계산합니다.
        """
        current_pos = np.array(current_pos)
        target_pos = np.array(target_pos)
        
        # 1. 목표를 향한 유도(Attractive) 벡터 계산
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        
        if distance < 0.1: # 목표 지점에 거의 도달한 경우
            return np.zeros(2)
            
        desired_vel = (direction / distance) * min(self.max_speed, distance)

        # 2. 이웃 기체와의 충돌 회피(Repulsive) 벡터 계산
        avoidance_vel = np.zeros(2)
        if neighbor_positions is not None:
            for neighbor in neighbor_positions:
                diff = current_pos - np.array(neighbor)
                dist = np.linalg.norm(diff)
                if 0 < dist < self.avoidance_radius:
                    # 거리가 가까울수록 강하게 밀어내는 힘 적용
                    avoidance_vel += (diff / dist) * (self.avoidance_radius - dist) * self.repulsive_gain

        # 3. 최종 속도 벡터 합성 및 클리핑
        final_vel = desired_vel + avoidance_vel
        speed = np.linalg.norm(final_vel)
        if speed > self.max_speed:
            final_vel = (final_vel / speed) * self.max_speed
            
        return final_vel