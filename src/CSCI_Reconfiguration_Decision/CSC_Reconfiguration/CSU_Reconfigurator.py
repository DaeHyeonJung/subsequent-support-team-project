import random
from typing import List
from src.CSCI_Simulation_Engine.CSC_Models.CSU_UavState import UavState

def trigger_reconfiguration_event(uavs: List[UavState], kill_count: int = 3) -> List[UavState]:
    """
    팀원(랜덤 기체 손실 알고리즘)과 본인(편대 재편성 및 할당)의 작업을 연결하기 위한 모듈입니다.
    지정된 수만큼 기체를 랜덤하게 손실(제거)시키고, 
    남은 기체들을 하나의 편대(formation_id=1)로 통합하여 반환합니다.
    """
    if len(uavs) <= kill_count:
        return []
        
    # 1. 랜덤 기체 손실 (추후 팀원분의 생존 판단 알고리즘이 적용될 위치)
    surviving_uavs = random.sample(uavs, len(uavs) - kill_count)
    
    # 2. 남은 기체들을 하나의 편대로 재편성 (formation_id 통일)
    for uav in surviving_uavs:
        uav.formation_id = 1
        
    return surviving_uavs