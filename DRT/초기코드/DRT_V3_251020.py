import math
import random
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import copy

# --- 0. 시뮬레이션 상수 정의 ---
SIMULATION_TIME = 2000
VEHICLE_SPEED = 4.0
VEHICLE_CAPACITY = 4
NUM_EXISTING_PASSENGERS = 4
NUM_NEW_PASSENGERS = 100
NEW_REQUEST_INTERVAL = 5
STATUS_PRINT_INTERVAL = 100

W_COST_INCREASE = 0.7
W_PATH_LENGTH = 0.3

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

# ⭐ 개선점 1: 용량 검사 함수가 현재 탑승객 수를 고려하도록 수정
def is_capacity_valid(path, capacity, current_onboard_count):
    onboard_count = current_onboard_count
    for point in path:
        if point.type == 'pickup':
            onboard_count += 1
        elif point.type == 'dropoff':
            onboard_count -= 1
        if onboard_count > capacity:
            return False
    return True

class Vehicle:
    def __init__(self, id, start_x, start_y, color, capacity):
        self.id = id
        self.current_location = Point(f'V{id}_current', start_x, start_y)
        self.path = []
        self.trajectory = [ (start_x, start_y) ]
        self.color = color
        self.capacity = capacity
        # ⭐ 개선점 2: 현재 탑승객 수를 직접 추적하는 변수 추가
        self.onboard_passengers = 0

# --- 2. 핵심 로직 함수 ---
def move_vehicles(vehicles, current_time):
    for v in vehicles:
        if not v.path: continue
        
        distance_can_move = VEHICLE_SPEED
        while distance_can_move > 0 and v.path:
            next_stop = v.path[0]
            dist_to_next = calculate_distance(v.current_location, next_stop)

            if dist_to_next <= distance_can_move:
                # ⭐ 개선점 3: 목적지 도착 시 탑승객 수 실시간 업데이트
                arrived_point = v.path.pop(0)
                if arrived_point.type == 'pickup':
                    v.onboard_passengers += 1
                elif arrived_point.type == 'dropoff':
                    v.onboard_passengers -= 1
                
                v.current_location = arrived_point
                print(f"  [Time: {current_time}] [Move] 차량 {v.id}, {arrived_point.id} 도착! (현재 탑승객: {v.onboard_passengers}명)")
                distance_can_move -= dist_to_next
            else:
                dx = next_stop.x - v.current_location.x
                dy = next_stop.y - v.current_location.y
                ratio = distance_can_move / dist_to_next
                v.current_location.x += dx * ratio
                v.current_location.y += dy * ratio
                distance_can_move = 0
        
        v.trajectory.append((v.current_location.x, v.current_location.y))


def assign_passenger_to_vehicle(vehicles, pickup, dropoff):
    best_vehicle = None
    best_new_path = []
    min_final_cost = float('inf')

    for v in vehicles:
        original_distance = calculate_path_distance(v.current_location, v.path)
        
        # 이미 현재 탑승객이 최대 용량이라면, 더 태울 수 없으므로 다음 차량 검사
        if v.onboard_passengers >= v.capacity:
            continue

        best_temp_path_for_vehicle = None
        min_temp_dist = float('inf')

        best_i = -1
        temp_min_dist_p = float('inf')
        
        for i in range(len(v.path) + 1):
            temp_path = v.path[:]
            temp_path.insert(i, pickup)
            dist = calculate_path_distance(v.current_location, temp_path)
            if dist < temp_min_dist_p:
                temp_min_dist_p = dist
                best_i = i
        
        path_with_pickup = v.path[:]
        path_with_pickup.insert(best_i, pickup)

        for j in range(best_i + 1, len(path_with_pickup) + 1):
            temp_path = path_with_pickup[:]
            temp_path.insert(j, dropoff)
            
            # ⭐ 개선점 1-1: 수정된 용량 검사 함수 호출
            if not is_capacity_valid(temp_path, v.capacity, v.onboard_passengers):
                continue

            dist = calculate_path_distance(v.current_location, temp_path)
            if dist < min_temp_dist:
                min_temp_dist = dist
                best_temp_path_for_vehicle = temp_path
        
        if not best_temp_path_for_vehicle:
            continue

        new_distance = min_temp_dist
        cost_increase = new_distance - original_distance
        final_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_distance)

        if final_cost < min_final_cost:
            min_final_cost = final_cost
            best_vehicle = v
            best_new_path = best_temp_path_for_vehicle
            
    return best_vehicle, best_new_path

def print_vehicle_status(v, title=""):
    if title: print(title)
    loc = v.current_location
    path_ids = [p.id for p in v.path]
    next_stop_id = path_ids[0] if v.path else "대기 중"
    
    # ⭐ 개선점 4: 상태 출력 시에도 실제 변수 값을 사용
    print(f"    - 현재 위치: ({loc.x:.1f}, {loc.y:.1f}) | 탑승객: {v.onboard_passengers}/{v.capacity}명 | 다음 목적지: {next_stop_id}")
    print(f"    - 남은 경로: {path_ids}")


# --- 3. 메인 시뮬레이션 실행 부 ---
def run_simulation():
    vehicles = [
        Vehicle(id=1, start_x=0, start_y=50, color='red', capacity=VEHICLE_CAPACITY),
        Vehicle(id=2, start_x=100, start_y=50, color='green', capacity=VEHICLE_CAPACITY)
    ]
    all_passengers = []
    
    # 초기 승객은 탑승 전이므로 onboard_passengers는 0으로 시작
    for i in range(NUM_EXISTING_PASSENGERS):
        pickup = Point(f'P{i+1}_Start', random.uniform(0, 100), random.uniform(0, 50), 'pickup')
        dropoff = Point(f'P{i+1}_End', random.uniform(0, 100), random.uniform(50, 100), 'dropoff')
        all_passengers.extend([pickup, dropoff])
        (vehicles[0] if i % 2 == 0 else vehicles[1]).path.extend([pickup, dropoff])
    
    new_passenger_requests = []
    for i in range(NUM_NEW_PASSENGERS):
        pickup = Point(f'NewP{i+1}_Start', random.uniform(20, 80), random.uniform(20, 80), 'pickup')
        dropoff = Point(f'NewP{i+1}_End', random.uniform(20, 80), random.uniform(20, 80), 'dropoff')
        new_passenger_requests.append((pickup, dropoff))

    simulation_history = []
    new_request_idx = 0

    print("="*40)
    print("      DRT 시뮬레이션 (개선 버전) 로그 시작")
    print("="*40)
    print("\n--- [Time: 0] 시뮬레이션 초기 상태 ---")
    for v in vehicles: print_vehicle_status(v, title=f"  차량 {v.id} 초기 상태:")

    for t in range(1, SIMULATION_TIME + 1):
        if t % NEW_REQUEST_INTERVAL == 0 and new_request_idx < NUM_NEW_PASSENGERS:
            pickup, dropoff = new_passenger_requests[new_request_idx]
            all_passengers.extend([pickup, dropoff])
            loc_info = f"(출발: ({pickup.x:.1f}, {pickup.y:.1f}), 도착: ({dropoff.x:.1f}, {dropoff.y:.1f}))"
            print(f"\n--- [Time: {t}] 신규 요청: {pickup.id} {loc_info} ---")
            
            assigned_vehicle, updated_path = assign_passenger_to_vehicle(vehicles, pickup, dropoff)
            
            if assigned_vehicle:
                assigned_vehicle.path = updated_path
                print_vehicle_status(assigned_vehicle, title=f"  -> 차량 {assigned_vehicle.id} 배정 완료 및 경로 업데이트:")
            else:
                print(f"  -> 배차 실패 (용량 또는 경로 문제)")
            new_request_idx += 1

        move_vehicles(vehicles, t)

        if t % STATUS_PRINT_INTERVAL == 0:
            print(f"\n--- [Time: {t}] 모든 차량 현재 상태 ---")
            for v in vehicles: print_vehicle_status(v, title=f"  차량 {v.id} 상태:")

        # deepcopy를 통해 차량의 모든 상태(onboard_passengers 포함)를 정확히 저장
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
            if p.type == 'pickup': 
                ax.plot(p.x, p.y, 'bo', markersize=6, alpha=0.5)
            else: 
                ax.plot(p.x, p.y, 'bx', markersize=6, alpha=0.5)
        
        for v in vehicles_state:
            traj_x, traj_y = zip(*v.trajectory)
            ax.plot(traj_x, traj_y, '-', color=v.color, alpha=0.3)
            ax.plot(v.current_location.x, v.current_location.y, 'o', color=v.color, markersize=12, label=f'Vehicle {v.id}')
            
            # ⭐ 개선점 5: 시각화 시에도 실제 변수 값을 사용
            onboard_passengers = v.onboard_passengers
            
            ax.text(v.current_location.x, v.current_location.y, str(onboard_passengers),
                    color='white', ha='center', va='center', fontweight='bold', fontsize=9)
            
            if v.path:
                path_points = [v.current_location] + v.path
                path_x, path_y = zip(*[(p.x, p.y) for p in path_points])
                ax.plot(path_x, path_y, '--', color=v.color, alpha=0.7)

        ax.set_xlim(0, 100); ax.set_ylim(0, 100)
        ax.set_title(f"DRT Routing Simulation (Improved) | Time: {frame + 1}/{len(history)}")
        ax.legend(); ax.grid(True)

    ani = FuncAnimation(fig, update, frames=len(history), interval=100, repeat=False)
    plt.show()

# --- 5. 프로그램 실행 ---
if __name__ == "__main__":
    random.seed(42)
    start_time = time.time()
    
    history = run_simulation()
    end_time = time.time()
    
    print("\n" + "="*40)
    print("               시뮬레이션 성능 결과")
    print("="*40)
    print(f"총 {SIMULATION_TIME} 단위시간 시뮬레이션 완료.")
    print(f"실제 소요 시간: {end_time - start_time:.4f} 초")
    
    print("\n이제 시뮬레이션 시각화를 시작합니다...")
    visualize_simulation(history)
    print("\n프로그램이 종료되었습니다.")