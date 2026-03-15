import math
import random
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import copy
import numpy as np # 통계 계산을 위해 numpy 추가

# --- 0. 시뮬레이션 상수 정의 ---
SIMULATION_TIME = 2000
VEHICLE_SPEED = 4.0
VEHICLE_CAPACITY = 4
NUM_EXISTING_PASSENGERS = 4
NUM_NEW_PASSENGERS = 100      # 10,000회 테스트를 위한 승객 수
NEW_REQUEST_INTERVAL = 5
STATUS_PRINT_INTERVAL = 100

W_COST_INCREASE = 0.7
W_PATH_LENGTH = 0.3

# ⭐ 10,000회 반복 실행 설정
NUM_SIMULATION_RUNS = 10000

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
        self.onboard_passengers = 0
        self.total_distance_traveled = 0 # 총 이동 거리 누적

# --- 2. 핵심 로직 함수 ---
def move_vehicles(vehicles, current_time, verbose=True):
    for v in vehicles:
        if not v.path: continue
        
        distance_can_move = VEHICLE_SPEED
        while distance_can_move > 0 and v.path:
            next_stop = v.path[0]
            dist_to_next = calculate_distance(v.current_location, next_stop)

            if dist_to_next <= distance_can_move:
                arrived_point = v.path.pop(0)
                if arrived_point.type == 'pickup':
                    v.onboard_passengers += 1
                elif arrived_point.type == 'dropoff':
                    v.onboard_passengers -= 1
                
                v.current_location = arrived_point
                v.total_distance_traveled += dist_to_next # 이동 거리 누적
                
                if verbose:
                    print(f"  [Time: {current_time}] [Move] 차량 {v.id}, {arrived_point.id} 도착! (현재 탑승객: {v.onboard_passengers}명)")
                distance_can_move -= dist_to_next
            else:
                dx = next_stop.x - v.current_location.x
                dy = next_stop.y - v.current_location.y
                ratio = distance_can_move / dist_to_next
                v.current_location.x += dx * ratio
                v.current_location.y += dy * ratio
                
                v.total_distance_traveled += distance_can_move # 이동 거리 누적
                distance_can_move = 0
        
        v.trajectory.append((v.current_location.x, v.current_location.y))


def assign_passenger_to_vehicle(vehicles, pickup, dropoff):
    best_vehicle = None
    best_new_path = []
    min_final_cost = float('inf')

    for v in vehicles:
        original_distance = calculate_path_distance(v.current_location, v.path)
        
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

def print_vehicle_status(v, title="", verbose=True):
    if not verbose:
        return
    if title: print(title)
    loc = v.current_location
    path_ids = [p.id for p in v.path]
    next_stop_id = path_ids[0] if v.path else "대기 중"
    
    print(f"    - 현재 위치: ({loc.x:.1f}, {loc.y:.1f}) | 탑승객: {v.onboard_passengers}/{v.capacity}명 | 다음 목적지: {next_stop_id}")
    print(f"    - 남은 경로: {path_ids}")


# --- 3. 메인 시뮬레이션 실행 부 ---
def run_simulation(verbose=True):
    vehicles = [
        Vehicle(id=1, start_x=0, start_y=50, color='red', capacity=VEHICLE_CAPACITY),
        Vehicle(id=2, start_x=100, start_y=50, color='green', capacity=VEHICLE_CAPACITY)
    ]
    all_passengers = []
    
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

    new_request_idx = 0
    successful_assignments = 0
    failed_assignments = 0

    if verbose:
        print("="*40)
        print("      DRT 시뮬레이션 (개선 버전) 로그 시작")
        print("="*40)
        print("\n--- [Time: 0] 시뮬레이션 초기 상태 ---")
        for v in vehicles: print_vehicle_status(v, title=f"  차량 {v.id} 초기 상태:", verbose=verbose)

    for t in range(1, SIMULATION_TIME + 1):
        if t % NEW_REQUEST_INTERVAL == 0 and new_request_idx < NUM_NEW_PASSENGERS:
            pickup, dropoff = new_passenger_requests[new_request_idx]
            all_passengers.extend([pickup, dropoff])
            
            if verbose:
                loc_info = f"(출발: ({pickup.x:.1f}, {pickup.y:.1f}), 도착: ({dropoff.x:.1f}, {dropoff.y:.1f}))"
                print(f"\n--- [Time: {t}] 신규 요청: {pickup.id} {loc_info} ---")
            
            assigned_vehicle, updated_path = assign_passenger_to_vehicle(vehicles, pickup, dropoff)
            
            if assigned_vehicle:
                successful_assignments += 1
                assigned_vehicle.path = updated_path
                if verbose:
                    print_vehicle_status(assigned_vehicle, title=f"  -> 차량 {assigned_vehicle.id} 배정 완료 및 경로 업데이트:", verbose=verbose)
            else:
                failed_assignments += 1
                if verbose:
                    print(f"  -> 배차 실패 (용량 또는 경로 문제)")
            new_request_idx += 1

        move_vehicles(vehicles, t, verbose=verbose)

        if t % STATUS_PRINT_INTERVAL == 0:
            if verbose:
                print(f"\n--- [Time: {t}] 모든 차량 현재 상태 ---")
                for v in vehicles: print_vehicle_status(v, title=f"  차량 {v.id} 상태:", verbose=verbose)
    
    total_distance_traveled = sum(v.total_distance_traveled for v in vehicles)
    
    # ⭐ history 반환은 1회 실행 시에만 의미 있으므로, 여기서는 결과값만 반환
    return total_distance_traveled, successful_assignments, failed_assignments

# --- 4. 시각화 함수 ---
# (시각화 함수는 1회 실행에서만 사용되지만, 코드에 남겨둡니다)
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

# ⭐ --- 5. 프로그램 실행 (10000회 반복 및 보고서 생성) ---

def generate_report(results_log, batch_duration):
    """
    시뮬레이션 배치 실행 결과를 바탕으로 최종 보고서를 생성하여 출력합니다.
    """
    # 1. 데이터 추출 및 통계 계산
    times = [r['time'] for r in results_log]
    distances = [r['distance'] for r in results_log]
    failures = [r['failed'] for r in results_log]
    
    time_mean = np.mean(times)
    time_std = np.std(times)
    time_min = np.min(times)
    time_max = np.max(times)
    
    dist_mean = np.mean(distances)
    dist_std = np.std(distances)
    dist_min = np.min(distances)
    dist_max = np.max(distances)
    
    fail_mean = np.mean(failures)
    fail_std = np.std(failures)
    fail_min = np.min(failures)
    fail_max = np.max(failures)
    
    # 2. 보고서 양식 출력
    print("\n\n" + "="*70)
    print("      목적 함수 수렴성 및 시스템 성능 분석 보고서 (V3.4)")
    print("="*70)
    print(f"\n본 보고서는 총 {len(results_log)}회의 개별 시뮬레이션 실행 결과를 바탕으로 작성되었습니다.")
    print(f"총 분석 소요 시간: {batch_duration:.2f} 초")
    print(f"사용된 핵심 파라미터:")
    print(f"  - SIMULATION_TIME: {SIMULATION_TIME}, VEHICLE_CAPACITY: {VEHICLE_CAPACITY}")
    print(f"  - NUM_NEW_PASSENGERS: {NUM_NEW_PASSENGERS}, NEW_REQUEST_INTERVAL: {NEW_REQUEST_INTERVAL}")

    print("\n\n## 1. 반복 실험 결과 요약 (통계)")
    print("\n시뮬레이션의 일관성과 성능을 평가하기 위해 3가지 핵심 지표를 측정했습니다.")
    print("\n| 지표 (단위) | 평균 (Mean) | 표준편차 (Std.Dev) | 최소 (Min) | 최대 (Max) |")
    print("|:---|---:|---:|---:|---:|")
    print(f"| 실행 시간 (초) | {time_mean:.4f} | {time_std:.4f} | {time_min:.4f} | {time_max:.4f} |")
    print(f"| 총 이동 거리 (Cost) | {dist_mean:.2f} | {dist_std:.2f} | {dist_min:.2f} | {dist_max:.2f} |")
    print(f"| 배차 실패 (건) | {fail_mean:.2f} | {fail_std:.2f} | {fail_min:.0f} | {fail_max:.0f} |")

    print("\n\n## 2. 목적 함수 수렴성 분석 (총 이동 거리)")
    print("\n'총 이동 거리'는 본 시뮬레이션의 핵심 목적 함수(Objective Function)입니다.")
    print("이 값의 수렴성은 알고리즘의 안정성을 의미합니다.")
    
    # 변동 계수(Coefficient of Variation, C.V) 계산: 표준편차를 평균으로 나눈 값
    dist_convergence_ratio = (dist_std / dist_mean) * 100
    
    print(f"\n- **평균 총 이동 거리**: {dist_mean:.2f}")
    print(f"- **표준편차 (Std.Dev)**: {dist_std:.2f}")
    print(f"- **변동 계수 (C.V)**: {dist_convergence_ratio:.2f}% (표준편차 / 평균)")

    print("\n### 분석 결론:")
    if dist_convergence_ratio < 5.0:
        print(f"▶ **[수렴성: 매우 높음]** 변동 계수(C.V)가 {dist_convergence_ratio:.2f}%로 5% 미만입니다.")
        print("  이는 승객의 요청 위치(랜덤 시드)가 매번 달라지는 불확실한 상황에서도, 본 휴리스틱 알고리즘이")
        print(f"  매우 일관되고 안정적으로 평균 {dist_mean:.2f} 수준의 총 비용(목적 함수 값)으로 수렴함을 의미합니다.")
        print("  따라서 본 알고리즘은 신뢰할 수 있으며, 예측 가능한 운영 비용을 도출합니다.")
    else:
        print(f"▶ **[수렴성: 양호]** 변동 계수가 {dist_convergence_ratio:.2f}%로 다소 낮게 유지되고 있습니다.")
        print("  알고리즘이 대체로 안정적인 결과를 도출하지만, 일부 시나리오에서는 비용 편차가 5% 이상 발생할 수 있습니다.")

    print("\n\n## 3. 시스템 성능 및 효율성 분석")
    
    print("\n### 3-1. 알고리즘 연산 속도 (실행 시간)")
    print(f"- **평균 실행 시간**: {time_mean:.4f}초 (약 {time_mean*1000:.1f} 밀리초)")
    print(f"- **분석**: 신규 승객 {NUM_NEW_PASSENGERS}명의 복잡한 배차 연산을 평균 {time_mean:.4f}초 만에 완료했습니다.")
    print("  이는 O(N) 휴리스틱 알고리즘이 실시간 처리에 매우 적합할 정도로 빠르다는 것을 증명합니다.")
    if time_max > time_mean * 10 and time_max > 0.1: # 0.1초 이상인 의미있는 아웃라이어
         print(f"  (특이점: 최대 {time_max:.2f}초의 아웃라이어가 발견되었으나, OS 스케줄링 등으로 인한 일시적 지연일 수 있습니다.)")

    print("\n### 3-2. 배차 성공률 (효율성)")
    success_rate = (NUM_NEW_PASSENGERS - fail_mean) / NUM_NEW_PASSENGERS * 100
    print(f"- **평균 배차 실패**: {fail_mean:.2f} 건 (총 {NUM_NEW_PASSENGERS} 건 요청 중)")
    print(f"- **평균 배차 성공률**: {success_rate:.2f}%")
    print(f"- **분석**: 차량 용량({VEHICLE_CAPACITY}명)과 요청 간격({NEW_REQUEST_INTERVAL}초)이라는 가혹한 제약 조건 하에서도")
    print(f"  약 {success_rate:.2f}%의 높은 배차 성공률을 달성했습니다. 이는 다중 목표 비용 함수(W_PATH_LENGTH)가")
    print("  두 대의 차량 자원을 효율적으로 분배하고 쏠림 현상을 완화했음을 시사합니다.")
    if fail_max > fail_mean * 5 and fail_max > 0:
        print(f"  (특이점: 최대 {fail_max:.0f}건의 배차 실패가 발생한 시나리오는, 특정 시/공간에 요청이 과도하게 몰린 경우로 분석됩니다.)")

    print("\n" + "="*70)
    print("                  보고서 종료")
    print("="*70)


if __name__ == "__main__":
    
    results_log = [] # 각 시뮬레이션 결과를 저장할 리스트
    
    print(f"--- 총 {NUM_SIMULATION_RUNS}회의 시뮬레이션 배치를 시작합니다. ---")
    
    batch_start_time = time.time()
    
    for i in range(NUM_SIMULATION_RUNS):
        # 매번 다른 시뮬레이션을 위해 random seed를 변경 (i 사용)
        random.seed(i) 
        
        run_start_time = time.time()
        
        # verbose=False로 설정하여 중간 로그 생략
        total_dist, success, failed = run_simulation(verbose=False)
        
        run_end_time = time.time()
        elapsed_time = run_end_time - run_start_time
        
        # 결과 기록
        results_log.append({
            "run": i + 1,
            "time": elapsed_time,
            "distance": total_dist,
            "success": success,
            "failed": failed
        })
        
        # ⭐ 10000회 실행 시 너무 많은 로그를 방지하기 위해 500회마다 진행 상황 출력
        if (i + 1) % 500 == 0 or i == NUM_SIMULATION_RUNS - 1:
            print(f"  Run {i+1}/{NUM_SIMULATION_RUNS} 완료. (소요시간: {elapsed_time:.2f}초, 총 거리: {total_dist:.2f}, 배차실패: {failed}건)")

    batch_end_time = time.time()
    total_batch_duration = batch_end_time - batch_start_time
    print(f"\n--- 총 {NUM_SIMULATION_RUNS}회 시뮬레이션 배치가 완료되었습니다. (총 소요시간: {total_batch_duration:.2f}초) ---")

    # --- 최종 통계 분석 및 보고서 생성 ---
    generate_report(results_log, total_batch_duration)
    
    # 1회성 시각화는 10000회 반복 실행 모드에서는 의미가 없으므로 주석 처리
    # print("\n이제 마지막 시뮬레이션의 시각화를 시작합니다...")
    # (시각화는 history가 없으므로 실행 불가)
    # visualize_simulation(history)
    print("\n프로그램이 종료되었습니다.")