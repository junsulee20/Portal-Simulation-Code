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
    for vehicle in vehicles:
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
    NUM_NEW_PASSENGERS = 4 # <--- 새로 타려는 승객 수를 4명으로 변경
    vehicles = [Vehicle(id=1), Vehicle(id=2)]
    
    print(f"--- 시뮬레이션 시작: 2개 차량에 기존 승객 {NUM_EXISTING_PASSENGERS}명이 탑승한 상황 ---")

    # 2. 기존 승객 10명의 경로를 2대 차량에 미리 할당
    for i in range(NUM_EXISTING_PASSENGERS):
        pickup = Point(f'P{i+1}_Start', random.uniform(0, 100), random.uniform(0, 100))
        dropoff = Point(f'P{i+1}_End', random.uniform(0, 100), random.uniform(0, 100))
        if (i+1) % 2 == 1:
            vehicles[0].path.extend([pickup, dropoff])
        else:
            vehicles[1].path.extend([pickup, dropoff])

    print("\n--- 초기 차량 상태 ---")
    for v in vehicles:
        print(f"차량 {v.id}의 초기 경로 (승객 {len(v.path)//2}명), 총 거리: {calculate_path_distance(v.path):.2f}")

    # 3. 새로운 승객 4명의 요청 목록 생성
    new_passenger_requests = []
    for i in range(NUM_NEW_PASSENGERS):
        pickup = Point(f'NewP{i+1}_Start', random.uniform(0, 100), random.uniform(0, 100))
        dropoff = Point(f'NewP{i+1}_End', random.uniform(0, 100), random.uniform(0, 100))
        new_passenger_requests.append((pickup, dropoff))

    print(f"\n--- 신규 승객 {NUM_NEW_PASSENGERS}명 순차 배정 시작 ---")

    # 4. 시간 측정 시작 (신규 승객 4명을 모두 배정하는 데 걸리는 시간 측정)
    start_time = time.time()
    
    # 5. 새로운 승객들을 순서대로 한 명씩 배정
    for i, (pickup_point, dropoff_point) in enumerate(new_passenger_requests):
        print(f"  [{i+1}/{NUM_NEW_PASSENGERS}] {pickup_point.id} 요청 처리 중...")
        
        assigned_vehicle, updated_path = assign_passenger_to_vehicle(vehicles, pickup_point, dropoff_point)
        
        if assigned_vehicle:
            # ✨ 중요: 배정된 차량의 경로를 즉시 업데이트해야 다음 승객 배정에 반영됨
            assigned_vehicle.path = updated_path
            print(f"   -> {pickup_point.id}는 차량 {assigned_vehicle.id}에 배정 완료.")
        else:
            print(f"   -> {pickup_point.id}는 배정 가능한 차량 없음.")

    end_time = time.time()
    elapsed_time = end_time - start_time

    # 6. 최종 결과 출력
    print("\n--- 최종 배정 결과 ---")
    for v in vehicles:
        path_str = [p.id for p in v.path]
        print(f"차량 {v.id}의 최종 경로 (총 승객 {len(v.path)//2}명)")
        print(f"  - 총 이동 거리: {calculate_path_distance(v.path):.2f}")

    print("\n--- 성능 측정 결과 ---")
    print(f"신규 승객 {NUM_NEW_PASSENGERS}명을 배정하는 데 걸린 총 소요 시간: {elapsed_time:.6f} 초")