import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import maximum_bipartite_matching


def fair_hungarian_assignment(cost_matrix: np.ndarray) -> np.ndarray:
    """
    정통 Fair 헝가리안 알고리즘 (Bottleneck Assignment Problem)
    이동 거리의 최댓값을 최소화(Minimax)하여, 가장 멀리 이동하는 기체의 희생(Unfairness)을 막습니다.
    
    :param cost_matrix: N x M 크기의 2차원 Numpy 배열 
                        (N: 남은 기체 수, M: 남은 슬롯 수, 일반적으로 N <= M)
    :return: 1차원 Numpy 배열 (인덱스: 기체 번호, 값: 할당된 슬롯 번호)
    """
    if cost_matrix.size == 0:
        return np.array([])

    # 1. 모든 고유한 비용(거리) 값을 추출하여 오름차순 정렬
    unique_costs = np.unique(cost_matrix)
    
    low = 0
    high = len(unique_costs) - 1
    best_matching = None
    
    # 2. 이진 탐색(Binary Search)을 통해 가능한 최소의 최대 거리(Threshold) 탐색
    while low <= high:
        mid = (low + high) // 2
        threshold = unique_costs[mid]
        
        # 3. threshold 이하의 비용을 가진 간선만 True(1)로 설정하여 이분 그래프 생성
        bipartite_graph = (cost_matrix <= threshold).astype(int)
        sparse_graph = csr_matrix(bipartite_graph)
        
        # 4. 최대 이분 매칭 수행 (row 기준 매칭 결과 반환)
        matching = maximum_bipartite_matching(sparse_graph, perm_type='row')
        
        # 5. 모든 기체(row)가 슬롯에 매칭되었는지 확인 (-1은 매칭 실패를 의미)
        if np.all(matching >= 0):
            best_matching = matching
            high = mid - 1
        else:
            low = mid + 1
            
    return best_matching