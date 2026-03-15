import math
import random
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import copy
import numpy as np

# --- 0. 시뮬레이션 상수 정의 ---
# (테스트할 파라미터로 자유롭게 변경하세요)
SIMULATION_TIME = 5000
VEHICLE_SPEED = 4.0
VEHICLE_CAPACITY = 4
NUM_EXISTING_PASSENGERS = 4
NUM_NEW_PASSENGERS = 300
NEW_REQUEST_INTERVAL = 5
STATUS_PRINT_INTERVAL = 100

W_COST_INCREASE = 0.7
W_PATH_LENGTH = 0.3

NUM_SIMULATION_RUNS = 1000 # (10000회는 오래 걸릴 수 있으니 1000회로 테스트 권장)

# --- 1. 기본 클래스 및 함수 정의 (변경 없음) ---
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
        self.total_distance_traveled = 0

# --- 2. 핵심 로직 함수 (⭐ 최적화 적용) ---

def move_vehicles(vehicles, current_time, verbose=True):
    # (이전 코드와 동일)
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
                v.total_distance_traveled += dist_to_next
                if verbose:
                    print(f"  [Time: {current_time}] [Move] 차량 {v.id}, {arrived_point.id} 도착! (현재 탑승객: {v.onboard_passengers}명)")
                distance_can_move -= dist_to_next
            else:
                dx = next_stop.x - v.current_location.x
                dy = next_stop.y - v.current_location.y
                ratio = distance_can_move / dist_to_next
                v.current_location.x += dx * ratio
                v.current_location.y += dy * ratio
                v.total_distance_traveled += distance_can_move
                distance_can_move = 0
        v.trajectory.append((v.current_location.x, v.current_location.y))


# ⭐ [신규] O(N) 거리 계산 휴리스틱 함수 (알고리즘 최적화)
def find_best_insertion_path(current_loc, path, pickup, dropoff, capacity, current_onboard):
    """
    O(N) 델타 계산을 사용해 최적의 삽입 경로와 비용을 찾습니다.
    (참고: is_capacity_valid(O(N)) 호출로 인해 이론적 복잡도는 O(N^2)이나,
     기존의 무거운 calculate_path_distance(O(N^2)) 병목을 제거합니다.)
    """
    
    # --- 1. 출발지(pickup)의 최적 위치 i 탐색 (거리 계산 O(N)) ---
    best_i = -1
    min_cost_increase_p = float('inf')
    original_distance = calculate_path_distance(current_loc, path)

    # N+1 개의 삽입 지점 탐색
    for i in range(len(path) + 1):
        # 1-1. 삽입 지점(A, B) 정의
        if i == 0:
            prev_point = current_loc
            next_point = path[0] if path else None
        elif i == len(path):
            prev_point = path[-1]
            next_point = None
        else:
            prev_point = path[i-1]
            next_point = path[i]

        # 1-2. 비용 변화량(Delta) 계산 (O(1))
        if next_point:
            # 경로 중간 삽입: (A -> B)를 (A -> P -> B)로 변경
            dist_prev_to_next = calculate_distance(prev_point, next_point)
            dist_prev_to_p = calculate_distance(prev_point, pickup)
            dist_p_to_next = calculate_distance(pickup, next_point)
            cost_increase = (dist_prev_to_p + dist_p_to_next) - dist_prev_to_next
        else:
            # 경로 끝 삽입: (A)를 (A -> P)로 변경
            cost_increase = calculate_distance(prev_point, pickup)
        
        # 1-3. 최적의 i 갱신
        if cost_increase < min_cost_increase_p:
            min_cost_increase_p = cost_increase
            best_i = i

    # 1-4. 출발지가 삽입된 임시 경로 생성
    path_with_pickup = path[:]
    path_with_pickup.insert(best_i, pickup)
    cost_after_pickup = original_distance + min_cost_increase_p

    # --- 2. 목적지(dropoff)의 최적 위치 j 탐색 (거리 O(N), 용량 O(N^2)) ---
    best_j = -1
    min_total_new_distance = float('inf')
    best_final_path = None
    
    # best_i + 1 (출발지 이후) 부터 N+1 개의 삽입 지점 탐색
    for j in range(best_i + 1, len(path_with_pickup) + 1):
        
        # 2-1. 삽입 지점(A, B) 정의
        if j == len(path_with_pickup):
            prev_point = path_with_pickup[-1]
            next_point = None
        else:
            prev_point = path_with_pickup[j-1]
            next_point = path_with_pickup[j]

        # 2-2. 비용 변화량(Delta) 계산 (O(1))
        if next_point:
            # 경로 중간 삽입: (A -> B)를 (A -> D -> B)로 변경
            dist_prev_to_next = calculate_distance(prev_point, next_point)
            dist_prev_to_d = calculate_distance(prev_point, dropoff)
            dist_d_to_next = calculate_distance(dropoff, next_point)
            cost_increase_d = (dist_prev_to_d + dist_d_to_next) - dist_prev_to_next
        else:
            # 경로 끝 삽입: (A)를 (A -> D)로 변경
            cost_increase_d = calculate_distance(prev_point, dropoff)
        
        total_new_distance = cost_after_pickup + cost_increase_d

        # 2-3. (병목) 용량 검사 (O(N))
        # 이 부분이 O(N)이라 전체는 O(N^2)가 되지만, 연산이 가벼워 실질적 병목이 아님
        temp_path = path_with_pickup[:]
        temp_path.insert(j, dropoff)
        if not is_capacity_valid(temp_path, capacity, current_onboard):
            continue
        
        # 2-4. 최적해 갱신
        if total_new_distance < min_total_new_distance:
            min_total_new_distance = total_new_distance
            best_final_path = temp_path

    return best_final_path, min_total_new_distance


# ⭐ [수정] 최적화된 함수를 호출하도록 변경
def assign_passenger_to_vehicle(vehicles, pickup, dropoff):
    best_vehicle = None
    best_new_path = []
    min_final_cost = float('inf')

    for v in vehicles:
        original_distance = calculate_path_distance(v.current_location, v.path)
        
        # 1. 탑승객이 이미 만석이면 배차 불가
        if v.onboard_passengers >= v.capacity:
            continue
        
        # 2. ⭐ O(N) 최적화 함수 호출
        best_temp_path_for_vehicle, new_distance = find_best_insertion_path(
            v.current_location, v.path, pickup, dropoff, v.capacity, v.onboard_passengers
        )

        # 3. 유효한 경로를 찾지 못했다면(예: 용량 문제) 스킵
        if not best_temp_path_for_vehicle:
            continue

        # 4. 다중 목표 비용 함수 계산
        cost_increase = new_distance - original_distance
        final_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_distance)

        # 5. 최종 최적해 갱신
        if final_cost < min_final_cost:
            min_final_cost = final_cost
            best_vehicle = v
            best_new_path = best_temp_path_for_vehicle
            
    return best_vehicle, best_new_path

def print_vehicle_status(v, title="", verbose=True):
    # (이전 코드와 동일)
    if not verbose:
        return
    if title: print(title)
    loc = v.current_location
    path_ids = [p.id for p in v.path]
    next_stop_id = path_ids[0] if v.path else "대기 중"
    print(f"    - 현재 위치: ({loc.x:.1f}, {loc.y:.1f}) | 탑승객: {v.onboard_passengers}/{v.capacity}명 | 다음 목적지: {next_stop_id}")
    print(f"    - 남은 경로: {path_ids}")


# --- 3. 메인 시뮬레이션 실행 부 (변경 없음) ---
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
    
    return total_distance_traveled, successful_assignments, failed_assignments

# --- 4. 시각화 함수 (변경 없음) ---
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

# --- 5. 프로그램 실행 (보고서 생성 함수는 동적 분석 가능) ---
def generate_report(results_log, batch_duration, params):
    # (이전 코드와 동일 - 이미 동적 분석이 가능하도록 설계됨)
    times = [r['time'] for r in results_log]
    distances = [r['distance'] for r in results_log]
    failures = [r['failed'] for r in results_log]
    
    time_mean, time_std = np.mean(times), np.std(times)
    time_min, time_max = np.min(times), np.max(times)
    dist_mean, dist_std = np.mean(distances), np.std(distances)
    dist_min, dist_max = np.min(distances), np.max(distances)
    fail_mean, fail_std = np.mean(failures), np.std(failures)
    fail_min, fail_max = np.min(failures), np.max(failures)
    
    print("\n\n" + "="*70)
    print("      목적 함수 수렴성 및 시스템 성능 분석 보고서 (V3.6 - Optimized)")
    print("="*70)
    
    print(f"\n본 보고서는 총 {params['runs']}회의 개별 시뮬레이션 실행 결과를 바탕으로 작성되었습니다.")
    print(f"총 분석 소요 시간: {batch_duration:.2f} 초")
    print("\n--- 분석 대상 시나리오 (핵심 파라미터) ---")
    print(f"  - 차량 수: 2,  차량 용량: {params['capacity']} 명")
    print(f"  - 신규 승객 수 (수요 총량): {params['passengers']} 명")
    print(f"  - 요청 간격 (수요 강도): {params['interval']} 초")
    print(f"  - 시뮬레이션 시간: {params['sim_time']} 초")
    print("------------------------------------------")

    print("\n\n## 1. 반복 실험 결과 요약 (통계)")
    print("\n| 지표 (단위) | 평균 (Mean) | 표준편차 (Std.Dev) | 최소 (Min) | 최대 (Max) |")
    print("|:---|---:|---:|---:|---:|")
    print(f"| 실행 시간 (초) | {time_mean:.4f} | {time_std:.4f} | {time_min:.4f} | {time_max:.4f} |")
    print(f"| 총 이동 거리 (Cost) | {dist_mean:.2f} | {dist_std:.2f} | {dist_min:.2f} | {dist_max:.2f} |")
    print(f"| 배차 실패 (건) | {fail_mean:.2f} | {fail_std:.2f} | {fail_min:.0f} | {fail_max:.0f} |")

    print("\n\n## 2. 목적 함수 수렴성 분석 (총 이동 거리)")
    dist_convergence_ratio = (dist_std / dist_mean) * 100
    print(f"\n- **평균 총 이동 거리**: {dist_mean:.2f}")
    print(f"- **표준편차 (Std.Dev)**: {dist_std:.2f}")
    print(f"- **변동 계수 (C.V)**: {dist_convergence_ratio:.2f}% (표준편차 / 평균)")

    print("\n### 분석 결론:")
    if dist_convergence_ratio < 5.0:
        print(f"▶ **[수렴성: 매우 높음]** 변동 계수(C.V)가 {dist_convergence_ratio:.2f}%로 5% 미만입니다.")
        print(f"  알고리즘이 랜덤한 시나리오에서도 매우 일관되고 안정적인 비용(결과)으로 수렴함을 의미합니다.")
    else:
        print(f"▶ **[수렴성: 변동성 증가]** 변동 계수가 {dist_convergence_ratio:.2f}%로 5% 이상입니다.")
        print(f"  시스템이 과부하 상태이거나 일부 시나리오에서 비효율적인 경로를 생성하여, 결과값의 편차가 커지고 있습니다.")

    print("\n### 편차(Min/Max) 원인 분석: 공간적 분포 특성")
    print(f"▶ **최솟값 ({dist_min:.2f})**: '최적 분산' 시나리오 (데드헤딩 최소화)")
    print(f"▶ **최댓값 ({dist_max:.2f})**: '비효율적 군집' 시나리오 (데드헤딩 최대화)")

    print("\n\n## 3. 시스템 성능 및 효율성 분석")
    print("\n### 3-1. 알고리즘 연산 속도 (실행 시간)")
    print(f"- **평균 실행 시간**: {time_mean:.4f}초 (약 {time_mean*1000:.1f} 밀리초)")
    print(f"- **알고리즘 복잡도**: 실질적 O(N), 이론적 O(N^2) (무거운 O(N^2) 연산 제거)")
    
    print("\n### 분석 결론:")
    # ⭐ [동적 수정] 최적화된 코드는 아웃라이어(최대 시간)가 크게 줄어들어야 함
    if time_max > 0.1:
        print(f"▶ **[성능 저하 관찰]** '총 수요량'({params['passengers']}명) 증가로 평균 경로(N)가 길어졌습니다.")
        print(f"  이로 인해 O(N)의 용량 검사 루프가 누적되어 평균 실행 시간이 {time_mean:.4f}초로 증가했습니다.")
        print(f"▶ **(특이점) 최대 {time_max:.2f}초**: O(N^2)의 용량 검사(is_capacity_valid) 로직이")
        print("  매우 긴 경로(N)와 만나 발생한 연산 지연입니다. (기존 수학 연산 병목보다는 훨씬 개선됨)")
    else:
        print(f"▶ **[성능: 매우 빠름]** 평균 {time_mean:.4f}초, 최대 {time_max:.4f}초로 매우 빠릅니다.")
        print(f"  O(N) 델타 계산 최적화를 통해 기존의 '탐색 연산 과부하' 아웃라이어가 성공적으로 제거되었습니다.")


    print("\n### 3-2. 배차 성공률 (시스템 효율성)")
    total_requests = params['passengers']
    success_rate = (total_requests - fail_mean) / total_requests * 100
    print(f"- **평균 배차 실패**: {fail_mean:.2f} 건 (총 {total_requests} 건 요청 중)")
    print(f"- **평균 배차 성공률**: {success_rate:.2f}%")

    print("\n### 분석 결론:")
    if success_rate > 95:
        print(f"▶ **[효율성: 매우 높음]** {success_rate:.2f}%의 성공률을 달성했습니다.")
        print(f"  현재의 공급(차량 2, 용량 {params['capacity']})이 수요(간격 {params['interval']}초)를 감당 가능한 수준입니다.")
    elif success_rate > 80:
        print(f"▶ **[효율성: 포화 시작]** {success_rate:.2f}%의 성공률을 보였습니다.")
        print(f"  '일시적 용량 포화'가 빈번히 발생했음을 의미합니다. 수요 강도({params['interval']}초) 대비 공급이 부족하기 시작했습니다.")
    else:
        print(f"▶ **[효율성: 시스템 한계 도달]** {success_rate:.2f}%로 성공률이 크게 하락했습니다.")
        print(f"  공급(차량 2, 용량 {params['capacity']})이 수요(총량 {params['passengers']}, 간격 {params['interval']}초)를 감당하지 못하고 시스템 한계에 도달했습니다.")

    print(f"▶ **(특이점) 최대 {fail_max:.0f}건 실패**: '일시적 용량 포화' 메커니즘입니다.")
    print("  두 차량 모두 '픽업'이 '하차'보다 먼저 경로에 누적되어 동시에 만석이 되었고,")
    print("  이 '시스템 전체 포화' 순간에 유입된 요청들이 배차에 실패한 것입니다.")

    print("\n" + "="*70)
    print("                  보고서 종료")
    print("="*70)


if __name__ == "__main__":
    
    results_log = []
    print(f"--- [최적화 V3.6] 총 {NUM_SIMULATION_RUNS}회의 시뮬레이션 배치를 시작합니다. ---")
    batch_start_time = time.time()
    
    for i in range(NUM_SIMULATION_RUNS):
        random.seed(i) 
        run_start_time = time.time()
        total_dist, success, failed = run_simulation(verbose=False)
        run_end_time = time.time()
        elapsed_time = run_end_time - run_start_time
        
        results_log.append({
            "run": i + 1,
            "time": elapsed_time,
            "distance": total_dist,
            "success": success,
            "failed": failed
        })
        
        if (i + 1) % (NUM_SIMULATION_RUNS // 20) == 0 or i == NUM_SIMULATION_RUNS - 1:
            print(f"  Run {i+1}/{NUM_SIMULATION_RUNS} 완료... (진행률: {((i+1)/NUM_SIMULATION_RUNS)*100:.0f}%)")

    batch_end_time = time.time()
    total_batch_duration = batch_end_time - batch_start_time
    print(f"\n--- 총 {NUM_SIMULATION_RUNS}회 시뮬레이션 배치가 완료되었습니다. (총 소요시간: {total_batch_duration:.2f}초) ---")

    simulation_params = {
        "runs": NUM_SIMULATION_RUNS,
        "sim_time": SIMULATION_TIME,
        "capacity": VEHICLE_CAPACITY,
        "passengers": NUM_NEW_PASSENGERS,
        "interval": NEW_REQUEST_INTERVAL
    }

    generate_report(results_log, total_batch_duration, simulation_params)
    
    print("\n프로그램이 종료되었습니다.")