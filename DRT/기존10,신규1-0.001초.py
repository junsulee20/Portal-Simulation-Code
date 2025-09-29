import math
import itertools
import time # 시간 측정을 위한 모듈
import random # 승객 데이터 생성을 위한 모듈

# 각 지점(승객 출발지, 목적지 등)의 좌표를 표현하는 클래스
class Point:
    def __init__(self, id, x, y):
        self.id = id  # 지점의 고유 ID (예: 'p1_start', 'p1_end')
        self.x = x
        self.y = y

# 두 지점 간의 유클리드 거리를 계산하는 함수
def calculate_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

# 주어진 경로(방문 순서)의 총 거리를 계산하는 함수
def calculate_path_distance(path):
    if not path:
        return 0
    total_dist = 0
    for i in range(len(path) - 1):
        total_dist += calculate_distance(path[i], path[i+1])
    return total_dist

# DRT 차량을 나타내는 클래스
class Vehicle:
    def __init__(self, id):
        self.id = id
        # 차량의 현재 경로 (방문해야 할 지점들의 순서)
        self.path = [] 

# 메인 라우팅 함수
def assign_passenger_to_vehicle(vehicles, new_passenger_pickup, new_passenger_dropoff):
    """
    새로운 승객 요청을 받아 최적의 차량에 배정하는 함수
    """
    best_vehicle = None
    best_new_path = []
    min_cost_increase = float('inf')

    # 모든 차량에 대해 반복하여 최적의 삽입 위치를 찾음
    for vehicle in vehicles: # <--- 여기가 수정된 부분입니다.
        original_distance = calculate_path_distance(vehicle.path)
        
        # 경로가 길어질수록 이 루프의 반복 횟수가 크게 증가함
        for i, j in itertools.permutations(range(len(vehicle.path) + 1), 2):
            if i > j:
                continue

            temp_path = vehicle.path[:]
            temp_path.insert(i, new_passenger_pickup)
            temp_path.insert(j + 1, new_passenger_dropoff)
            new_distance = calculate_path_distance(temp_path)
            cost_increase = new_distance - original_distance
            
            if cost_increase < min_cost_increase:
                min_cost_increase = cost_increase
                best_vehicle = vehicle
                best_new_path = temp_path
                
    return best_vehicle, best_new_path


# --- 시뮬레이션 예제 ---
if __name__ == "__main__":
    # 1. 시뮬레이션 환경 설정
    NUM_EXISTING_PASSENGERS = 10
    vehicles = [Vehicle(id=1), Vehicle(id=2)]
    
    print(f"--- 시뮬레이션 시작: 2개 차량에 기존 승객 {NUM_EXISTING_PASSENGERS}명이 탑승한 상황 ---")

    # 2. 기존 승객 10명의 경로를 2대 차량에 미리 할당
    #    (차량 1에 5명, 차량 2에 5명)
    for i in range(NUM_EXISTING_PASSENGERS):
        pickup = Point(f'P{i+1}_Start', random.uniform(0, 100), random.uniform(0, 100))
        dropoff = Point(f'P{i+1}_End', random.uniform(0, 100), random.uniform(0, 100))
        
        # 간단하게 홀수 승객은 1번, 짝수 승객은 2번 차량에 배정
        # 실제로는 이 경로도 최적화되어 있겠지만, 여기서는 복잡한 경로를 가정하기 위해 임의로 추가
        if (i+1) % 2 == 1:
            vehicles[0].path.extend([pickup, dropoff])
        else:
            vehicles[1].path.extend([pickup, dropoff])

    print("\n--- 초기 차량 상태 ---")
    for v in vehicles:
        # 경로가 너무 길 경우 일부만 표시
        path_str = [p.id for p in v.path]
        if len(path_str) > 10:
            path_str = path_str[:5] + ['...'] + path_str[-5:]
        print(f"차량 {v.id}의 초기 경로 (승객 {len(v.path)//2}명): {path_str}")
        print(f"  - 총 이동 거리: {calculate_path_distance(v.path):.2f}")


    # 3. 새로운 승객(11번째) 요청 생성
    new_passenger_pickup = Point('NewP_Start', random.uniform(0, 100), random.uniform(0, 100))
    new_passenger_dropoff = Point('NewP_End', random.uniform(0, 100), random.uniform(0, 100))
    
    print(f"\n--- 신규 요청: {new_passenger_pickup.id} -> {new_passenger_dropoff.id} 배정 시작 ---")

    # 4. 시간 측정 시작 (신규 승객 1명을 배정하는 데 걸리는 시간만 측정)
    start_time = time.time()
    
    assigned_vehicle, updated_path = assign_passenger_to_vehicle(vehicles, new_passenger_pickup, new_passenger_dropoff)
    
    end_time = time.time()
    elapsed_time = end_time - start_time

    # 5. 최종 결과 출력
    print("\n--- 배정 및 성능 측정 결과 ---")
    if assigned_vehicle:
        print(f"신규 승객은 차량 {assigned_vehicle.id}에 배정되었습니다.")
        # 해당 차량의 경로 업데이트 (시뮬레이션)
        # assigned_vehicle.path = updated_path 
    else:
        print("배정 가능한 차량을 찾지 못했습니다.")
        
    print(f"소요 시간: {elapsed_time:.6f} 초")