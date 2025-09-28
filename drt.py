import math
import itertools

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
        # 예: [차량 현재 위치, 승객1 출발지, 승객2 출발지, 승객1 목적지, ...]
        self.path = [] 

# 메인 라우팅 함수
def assign_passenger_to_vehicle(vehicles, new_passenger_pickup, new_passenger_dropoff):
    """
    새로운 승객 요청을 받아 최적의 차량에 배정하는 함수

    Args:
        vehicles (list): 현재 운행 중인 모든 차량 객체의 리스트
        new_passenger_pickup (Point): 신규 승객의 출발지
        new_passenger_dropoff (Point): 신규 승객의 목적지

    Returns:
        tuple: (최적 차량 객체, 최적화된 새로운 경로) 또는 (None, None)
    """
    
    best_vehicle = None
    best_new_path = []
    min_cost_increase = float('inf') # 최소 추가 비용을 저장하기 위한 변수, 무한대로 초기화

    print(f"\n--- 신규 승객 요청: {new_passenger_pickup.id} -> {new_passenger_dropoff.id} ---")

    # 모든 차량에 대해 반복하여 최적의 삽입 위치를 찾음
    for vehicle in vehicles:
        original_distance = calculate_path_distance(vehicle.path) # 기존 경로의 총 거리
        
        # 승객의 출발지(p)와 목적지(d)를 삽입할 수 있는 모든 위치 조합을 생성
        # 예를 들어 경로가 [A, B] 라면, 삽입 가능한 인덱스는 0, 1, 2 (맨 앞, 사이, 맨 뒤)
        # itertools.permutations를 사용해 모든 삽입 위치 조합을 구함
        for i, j in itertools.permutations(range(len(vehicle.path) + 1), 2):
            
            # 출발지(i)가 목적지(j)보다 앞에 오거나 같은 위치여야 함
            if i > j:
                continue

            # 임시로 새로운 경로를 만들어 봄
            temp_path = vehicle.path[:] # 기존 경로 복사
            temp_path.insert(i, new_passenger_pickup) # i 위치에 출발지 삽입
            temp_path.insert(j + 1, new_passenger_dropoff) # j+1 위치에 목적지 삽입 (i에 삽입 후 리스트 길이가 1 늘어났기 때문)

            # 새로 만들어진 경로의 총 거리를 계산
            new_distance = calculate_path_distance(temp_path)
            
            # 추가된 비용(거리) 계산
            cost_increase = new_distance - original_distance
            
            print(f"  [차량 {vehicle.id}] 테스트 경로: {[p.id for p in temp_path]}, 추가 거리: {cost_increase:.2f}")

            # 현재까지의 최소 추가 비용보다 더 적은 비용이 드는 경우
            if cost_increase < min_cost_increase:
                min_cost_increase = cost_increase
                best_vehicle = vehicle
                best_new_path = temp_path
                
    print(f"--- 결과 ---")
    if best_vehicle:
        print(f"최적 차량: 차량 {best_vehicle.id}")
        print(f"최소 추가 거리: {min_cost_increase:.2f}")
        print(f"새로운 최적 경로: {[p.id for p in best_new_path]}")
    else:
        print("배정 가능한 차량을 찾지 못했습니다.")

    return best_vehicle, best_new_path


# --- 시뮬레이션 예제 ---
if __name__ == "__main__":
    # 2대의 차량 생성
    vehicle1 = Vehicle(id=1)
    vehicle2 = Vehicle(id=2)

    # 차량 1에 이미 배정된 승객(P1)의 경로가 있다고 가정
    p1_start = Point('P1_Start', 1, 1)
    p1_end = Point('P1_End', 5, 5)
    vehicle1.path = [p1_start, p1_end]

    # 차량 2는 다른 방향으로 가는 승객(P2)을 태움
    p2_start = Point('P2_Start', 10, 10)
    p2_end = Point('P2_End', 15, 12)
    vehicle2.path = [p2_start, p2_end]
    
    vehicles = [vehicle1, vehicle2]
    
    print("--- 초기 상태 ---")
    print(f"차량 1 경로: {[p.id for p in vehicle1.path]}, 총 거리: {calculate_path_distance(vehicle1.path):.2f}")
    print(f"차량 2 경로: {[p.id for p in vehicle2.path]}, 총 거리: {calculate_path_distance(vehicle2.path):.2f}")


    # 새로운 승객(NewP) 요청 발생 (차량 1의 경로와 유사한 방향)
    new_p_start = Point('NewP_Start', 2, 2)
    new_p_end = Point('NewP_End', 6, 6)

    # 최적 차량 탐색 및 배정
    assigned_vehicle, updated_path = assign_passenger_to_vehicle(vehicles, new_p_start, new_p_end)

    # 결과 업데이트
    if assigned_vehicle:
        assigned_vehicle.path = updated_path

    print("\n--- 최종 배정 후 상태 ---")
    print(f"차량 1 경로: {[p.id for p in vehicle1.path]}")
    print(f"차량 2 경로: {[p.id for p in vehicle2.path]}")