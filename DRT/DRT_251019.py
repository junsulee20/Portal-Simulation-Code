import math
import itertools
import random
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import copy # 객체의 깊은 복사를 위해 추가

# --- 0. 시뮬레이션 상수 정의 ---
SIMULATION_TIME = 1000       # 총 시뮬레이션 시간
VEHICLE_SPEED = 4.0         # 차량의 시간당 이동 속도
NUM_EXISTING_PASSENGERS = 20 # 기존 승객 수
NUM_NEW_PASSENGERS = 100     # 신규 승객 수
NEW_REQUEST_INTERVAL = 10   # 신규 승객 요청이 들어오는 시간 간격
STATUS_PRINT_INTERVAL = 100  # 텍스트로 현재 상태를 출력하는 시간 간격

# --- 1. 기본 클래스 및 함수 정의 ---
class Point:
    def __init__(self, id, x, y, type=''):
        self.id = id
        self.x = x
        self.y = y
        self.type = type

def calculate_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def calculate_path_distance(start_point, path):
    if not path: return 0
    total_dist = calculate_distance(start_point, path[0])
    for i in range(len(path) - 1):
        total_dist += calculate_distance(path[i], path[i+1])
    return total_dist

class Vehicle:
    def __init__(self, id, start_x, start_y, color):
        self.id = id
        self.current_location = Point(f'V{id}_current', start_x, start_y)
        self.path = []
        self.trajectory = [ (start_x, start_y) ]
        self.color = color

# --- 2. 핵심 로직 함수 ---
def move_vehicles(vehicles, current_time):
    for v in vehicles:
        if not v.path: continue
        next_stop = v.path[0]
        dist_to_next = calculate_distance(v.current_location, next_stop)
        if dist_to_next < VEHICLE_SPEED:
            v.current_location = next_stop
            print(f"  [Time: {current_time}] [Move] 차량 {v.id}, {v.path.pop(0).id} 목적지 도착!")
        else:
            dx = next_stop.x - v.current_location.x
            dy = next_stop.y - v.current_location.y
            ratio = VEHICLE_SPEED / dist_to_next
            v.current_location.x += dx * ratio
            v.current_location.y += dy * ratio
        v.trajectory.append((v.current_location.x, v.current_location.y))

def assign_passenger_to_vehicle(vehicles, pickup, dropoff):
    best_vehicle, best_new_path, min_cost_increase = None, [], float('inf')
    for v in vehicles:
        original_distance = calculate_path_distance(v.current_location, v.path)
        for i, j in itertools.permutations(range(len(v.path) + 1), 2):
            if i > j: continue
            temp_path = v.path[:]
            temp_path.insert(i, pickup)
            temp_path.insert(j + 1, dropoff)
            new_distance = calculate_path_distance(v.current_location, temp_path)
            cost_increase = new_distance - original_distance
            if cost_increase < min_cost_increase:
                min_cost_increase, best_vehicle, best_new_path = cost_increase, v, temp_path
    return best_vehicle, best_new_path

def print_vehicle_status(v, title=""):
    if title: print(title)
    loc, path_ids = v.current_location, [p.id for p in v.path]
    next_stop_id = path_ids[0] if v.path else "대기 중"
    print(f"    - 현재 위치: ({loc.x:.1f}, {loc.y:.1f}) | 다음 목적지: {next_stop_id}")
    print(f"    - 남은 경로: {path_ids}")

# --- 3. 메인 시뮬레이션 실행 부 ---
def run_simulation():
    # 시뮬레이션 객체 초기화
    vehicles = [
        Vehicle(id=1, start_x=0, start_y=50, color='red'),
        Vehicle(id=2, start_x=100, start_y=50, color='green')
    ]
    all_passengers = []
    for i in range(NUM_EXISTING_PASSENGERS):
        pickup = Point(f'P{i+1}_Start', random.uniform(0, 100), random.uniform(0, 50), 'pickup')
        dropoff = Point(f'P{i+1}_End', random.uniform(0, 100), random.uniform(50, 100), 'dropoff')
        all_passengers.extend([pickup, dropoff])
        (vehicles[0] if pickup.y < 25 else vehicles[1]).path.extend([pickup, dropoff])
    
    new_passenger_requests = []
    for i in range(NUM_NEW_PASSENGERS):
        pickup = Point(f'NewP{i+1}_Start', random.uniform(20, 80), random.uniform(20, 80), 'pickup')
        dropoff = Point(f'NewP{i+1}_End', random.uniform(20, 80), random.uniform(20, 80), 'dropoff')
        new_passenger_requests.append((pickup, dropoff))

    simulation_history = []
    new_request_idx = 0

    print("="*40)
    print("      DRT 시뮬레이션 텍스트 로그 시작")
    print("="*40)
    print("\n--- [Time: 0] 시뮬레이션 초기 상태 ---")
    for v in vehicles: print_vehicle_status(v, title=f"  차량 {v.id} 초기 상태:")

    # 메인 시뮬레이션 루프
    for t in range(1, SIMULATION_TIME + 1):
        # 신규 승객 요청 처리
        if t % NEW_REQUEST_INTERVAL == 0 and new_request_idx < NUM_NEW_PASSENGERS:
            pickup, dropoff = new_passenger_requests[new_request_idx]
            all_passengers.extend([pickup, dropoff])
            loc_info = f"(출발: ({pickup.x:.1f}, {pickup.y:.1f}), 도착: ({dropoff.x:.1f}, {dropoff.y:.1f}))"
            print(f"\n--- [Time: {t}] 신규 요청: {pickup.id} {loc_info} ---")
            assigned_vehicle, updated_path = assign_passenger_to_vehicle(vehicles, pickup, dropoff)
            if assigned_vehicle:
                assigned_vehicle.path = updated_path
                print_vehicle_status(assigned_vehicle, title=f"  -> 차량 {assigned_vehicle.id} 배정 완료 및 경로 업데이트:")
            new_request_idx += 1

        # 차량 이동
        move_vehicles(vehicles, t)

        # 현재 상태 텍스트 출력
        if t % STATUS_PRINT_INTERVAL == 0:
            print(f"\n--- [Time: {t}] 모든 차량 현재 상태 ---")
            for v in vehicles: print_vehicle_status(v, title=f"  차량 {v.id} 상태:")

        # 시각화를 위해 현재 상태 저장 (깊은 복사)
        current_state = {'vehicles': copy.deepcopy(vehicles), 'passengers': copy.deepcopy(all_passengers)}
        simulation_history.append(current_state)
    
    return simulation_history

# --- 4. 시각화 함수 ---
def visualize_simulation(history):
    fig, ax = plt.subplots(figsize=(10, 10))

    def update(frame):
        ax.clear()
        state = history[frame]
        vehicles_state = state['vehicles']
        passengers_state = state['passengers']

        for p in passengers_state:
            if p.type == 'pickup': ax.plot(p.x, p.y, 'bo', markersize=6, alpha=0.5)
            else: ax.plot(p.x, p.y, 'bx', markersize=6, alpha=0.5)
        
        for v in vehicles_state:
            traj_x, traj_y = zip(*v.trajectory)
            ax.plot(traj_x, traj_y, '-', color=v.color, alpha=0.3)
            ax.plot(v.current_location.x, v.current_location.y, 'o', color=v.color, markersize=12, label=f'Vehicle {v.id}')
            if v.path:
                path_points = [v.current_location] + v.path
                path_x, path_y = zip(*[(p.x, p.y) for p in path_points])
                ax.plot(path_x, path_y, '--', color=v.color, alpha=0.7)

        ax.set_xlim(0, 100); ax.set_ylim(0, 100)
        ax.set_title(f"DRT Routing Simulation | Time: {frame + 1}/{len(history)}")
        ax.legend(); ax.grid(True)

    ani = FuncAnimation(fig, update, frames=len(history), interval=100, repeat=False)
    plt.show()

# --- 5. 프로그램 실행 ---
if __name__ == "__main__":
    start_time = time.time()
    # 1. 시뮬레이션 실행 및 텍스트 로그 출력
    history = run_simulation()
    end_time = time.time()
    
    # 2. 성능 결과 출력
    print("\n" + "="*40)
    print("            시뮬레이션 성능 결과")
    print("="*40)
    print(f"총 {SIMULATION_TIME} 단위시간 시뮬레이션 완료.")
    print(f"실제 소요 시간: {end_time - start_time:.4f} 초")
    
    # 3. 시각화 창 띄우기
    print("\n이제 시뮬레이션 시각화를 시작합니다...")
    visualize_simulation(history)
    print("\n프로그램이 종료되었습니다.")