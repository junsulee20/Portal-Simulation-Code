"""
실제 운행 중인 DRT(화성똑버스03)를 재현하는 realized 시뮬레이션 모듈.

`drt_network_simulation_optimized.py`를 기반으로 하되, 향남신도시 지역으로 제한된 시뮬레이션입니다.

주요 특징:
    - 향남신도시 지역(향남1~2신도시, 발안리, 구문천리, 제암리 일대)으로 제한
    - 디멘드 20개, 차량 5대로 설정
    - 좌표 범위 기반 자동 노드 필터링
    - 고정된 디멘드와 차량 초기 위치 사용

성능 최적화:
    - 배정 단계에서 경로 노드 구축 제거 (시각화 단계로 지연)
    - 경로 계산 최적화 및 캐싱 활용
    - 불필요한 중간 경로 계산 최소화
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib import font_manager
import networkx as nx

# from drt_network_assignment_optimized import (
#     DRTAssignmentEngine,
#     NetworkTravelTimeCache,
#     PassengerRequest,
#     Stop,
#     VehicleState,
#     load_network_graph,
#     select_random_node,
# )

from drt_network_assignment_optimized_sampling_3항 import (
    DRTAssignmentEngine,
    MAX_DISPATCH_ETA_SECONDS,
    MAX_PATH_LENGTH,
    NetworkTravelTimeCache,
    PassengerRequest,
    Stop,
    VehicleState,
    load_network_graph,
    select_random_node,
)

# --------------------------------------------------------------------------------------
# ================================================================================
# 시뮬레이션 설정
# ================================================================================
NUM_DEMANDS = 40  # 일반 디멘드 개수
NUM_STREET_HAIL_DEMANDS = 10 # 추가: 길거리 디멘드 개수
NUM_VEHICLES = 5  # 차량 수
VEHICLE_CAPACITY = 14  # 차량 용량
REQUEST_INTERVAL_SECONDS = 30  # 각 디멘드 요청 간격 (초)
STREET_HAIL_INTERVAL_SECONDS = 60 # 추가: 길거리 디멘드 요청 간격 (초)
MAX_AWAIT_TIME_SECONDS = 600  # 일반 대기열(큐)에서 승객이 배차를 기다리는 최대 허용 시간 (초, 600초=10분)
STREET_HAIL_QUEUE_TIMEOUT_SECONDS = 300 # 추가: 길거리 대기열(큐) 최대 허용 시간 (초)
STREET_HAIL_TRAVEL_TIME_INCREASE_LIMIT = 300 # 추가: 기존 통행시간 증가 가드레일 (초)

# 시드 값 (고정된 디멘드 생성을 위해)
DEMAND_SEED = 42
STREET_HAIL_DEMAND_SEED = 1004
VEHICLE_SEED = 123

# ================================================================================
# 향남신도시 지역 좌표 범위 설정
# ================================================================================
# 사용자가 제공한 좌표 범위:
# - 왼쪽 아래: 37.107927, 126.903754
# - 오른쪽 아래: 37.140525, 126.903754
# - 왼쪽 위: 37.107927, 126.903436
# - 오른쪽 위: 37.140525, 126.929743
REGION_MIN_LAT = 37.107927  # 최소 위도
REGION_MAX_LAT = 37.140525  # 최대 위도
REGION_MIN_LON = 126.903436  # 최소 경도
REGION_MAX_LON = 126.929743  # 최대 경도
# ================================================================================
# --------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------
# Matplotlib 한글 폰트 설정
# --------------------------------------------------------------------------------------


def configure_matplotlib_font() -> None:
    preferred_fonts = [
        "Malgun Gothic",
        "NanumGothic",
        "Nanum Gothic",
        "AppleGothic",
        "NanumGothicCoding",
        "Noto Sans CJK KR",
        "Noto Sans KR",
    ]

    available_fonts = {f.name for f in font_manager.fontManager.ttflist}
    for font_name in preferred_fonts:
        if font_name in available_fonts:
            plt.rcParams["font.family"] = font_name
            break
    else:
        print("⚠️  한글 폰트를 찾지 못했습니다. 시스템에 한글 폰트를 설치하거나 rcParams를 수동 설정하세요.")

    plt.rcParams["axes.unicode_minus"] = False


configure_matplotlib_font()

PRESET_VEHICLE_COLORS: Dict[int, str] = {
    1: "#0077b6",
    2: "#ef476f",
}

FALLBACK_VEHICLE_COLORS: List[str] = [
    "#2a9d8f",
    "#f4a261",
    "#e76f51",
    "#8338ec",
    "#06d6a0",
    "#ffafcc",
]

# --------------------------------------------------------------------------------------
# 데이터 구조
# --------------------------------------------------------------------------------------


@dataclass
class AssignmentEvent:
    """단일 배차 이벤트 기록 (성능 최적화: 경로 노드는 시각화 시에만 계산)."""

    request: PassengerRequest
    vehicle_id: int
    vehicle_start_node: int
    cost: float
    cost_increase: float  # 추가: 목적함수 1항
    new_path_time: float  # 추가: 목적함수 2항
    total_wait: float     # 추가: 목적함수 3항
    wait_assign: float    # 추가: 대기시간(요청-배정)
    wait_pickup: float    # 추가: 대기시간(배정-픽업)
    previous_path: List[Stop]
    new_path: List[Stop]
    request_time: float  # 요청이 들어온 시각 (초)
    assignment_time: float  # 배정 완료 시각 (초)
    waiting_time: float  # 대기시간: 배정 완료까지 걸린 시간 (초)
    travel_time: float  # 통행시간: 픽업부터 드롭오프까지의 시간 (초)
    pickup_time: float  # 픽업 시각 (초)
    dropoff_time: float  # 드롭오프 시각 (초)
    straight_line_distance_degrees: float  # 픽업-드롭오프 직선 거리 (도 단위)
    straight_line_distance_km: float  # 픽업-드롭오프 직선 거리 (km 단위)
    speed_kmh: float  # 평균 속도 (km/h)
    # 경로 노드는 시각화 시에만 계산하므로 저장하지 않음


@dataclass
class FixedDemand:
    """고정된 디멘드 정보."""

    passenger_id: str
    pickup_node: int
    dropoff_node: int
    is_street_hail: bool = False
    request_interval: float = 30.0 # 요청 시뮬레이션 도착 간격


@dataclass
class FixedVehicleInitial:
    """고정된 차량 초기 위치."""

    vehicle_id: int
    initial_node: int


# --------------------------------------------------------------------------------------
# 지역 필터링 함수
# --------------------------------------------------------------------------------------


def filter_nodes_by_region(
    graph: nx.Graph,
    min_lon: float,
    max_lon: float,
    min_lat: float,
    max_lat: float,
) -> List[int]:
    """
    지정된 좌표 범위 내의 노드만 필터링하여 반환합니다.
    
    Args:
        graph: 네트워크 그래프
        min_lon: 최소 경도
        max_lon: 최대 경도
        min_lat: 최소 위도
        max_lat: 최대 위도
    
    Returns:
        필터링된 노드 ID 리스트
    """
    filtered_nodes = []
    
    for node in graph.nodes:
        node_data = graph.nodes.get(node)
        if not node_data:
            continue
        
        lon = node_data.get("longitude")
        lat = node_data.get("latitude")
        
        if lon is None or lat is None:
            continue
        
        # 좌표 범위 내에 있는지 확인
        if min_lon <= lon <= max_lon and min_lat <= lat <= max_lat:
            filtered_nodes.append(node)
    
    return filtered_nodes


# --------------------------------------------------------------------------------------
# 고정된 디멘드 및 차량 초기 위치 생성
# --------------------------------------------------------------------------------------


def generate_fixed_demands(
    graph: nx.Graph,
    num_demands: int,
    seed: int,
    allowed_nodes: Optional[List[int]] = None,
) -> List[FixedDemand]:
    """
    고정된 디멘드 목록을 생성합니다.
    
    Args:
        graph: 네트워크 그래프
        num_demands: 생성할 디멘드 개수
        seed: 랜덤 시드
        allowed_nodes: 허용된 노드 리스트 (None이면 모든 노드 사용)
    
    Returns:
        고정된 디멘드 목록
    """
    random_gen = random.Random(seed)
    
    # 허용된 노드 리스트 사용 (지정되지 않으면 모든 노드 사용)
    if allowed_nodes is None:
        nodes = list(graph.nodes)
    else:
        nodes = allowed_nodes
    
    if not nodes:
        raise ValueError("허용된 노드가 없습니다. 좌표 범위를 확인해주세요.")
    
    demands = []
    
    for idx in range(1, num_demands + 1):
        pickup = random_gen.choice(nodes)
        dropoff = random_gen.choice(nodes)
        while dropoff == pickup:
            dropoff = random_gen.choice(nodes)
        
        demands.append(
            FixedDemand(
                passenger_id=f"demand_{idx:03d}",
                pickup_node=pickup,
                dropoff_node=dropoff,
                is_street_hail=False,
                request_interval=REQUEST_INTERVAL_SECONDS
            )
        )
    
    return demands

def generate_street_hail_demands(
    graph: nx.Graph,
    num_demands: int,
    seed: int,
    allowed_nodes: Optional[List[int]] = None,
) -> List[FixedDemand]:
    """고정된 길거리 대기 디멘드 생성"""
    random_gen = random.Random(seed)
    nodes = allowed_nodes if allowed_nodes is not None else list(graph.nodes)
    
    demands = []
    for idx in range(1, num_demands + 1):
        pickup = random_gen.choice(nodes)
        dropoff = random_gen.choice(nodes)
        while dropoff == pickup:
            dropoff = random_gen.choice(nodes)
            
        demands.append(
            FixedDemand(
                passenger_id=f"street_{idx:03d}",
                pickup_node=pickup,
                dropoff_node=dropoff,
                is_street_hail=True,
                request_interval=STREET_HAIL_INTERVAL_SECONDS
            )
        )
    return demands


def generate_fixed_vehicle_initials(
    num_vehicles: int,
    graph: nx.Graph,
    seed: int,
    allowed_nodes: Optional[List[int]] = None,
) -> List[FixedVehicleInitial]:
    """
    고정된 차량 초기 위치를 생성합니다.
    모든 차량은 동일한 차고지(초기 위치)를 사용합니다.
    
    Args:
        num_vehicles: 차량 수
        graph: 네트워크 그래프
        seed: 랜덤 시드 (차고지 위치 선택용)
        allowed_nodes: 허용된 노드 리스트 (None이면 모든 노드 사용)
    
    Returns:
        고정된 차량 초기 위치 목록 (모든 차량이 동일한 노드 사용)
    """
    random_gen = random.Random(seed)
    
    # 허용된 노드 리스트 사용 (지정되지 않으면 모든 노드 사용)
    if allowed_nodes is None:
        nodes = list(graph.nodes)
    else:
        nodes = allowed_nodes
    
    if not nodes:
        raise ValueError("허용된 노드가 없습니다. 좌표 범위를 확인해주세요.")
    
    # 모든 차량이 동일한 차고지(초기 위치)를 사용
    depot_node = random_gen.choice(nodes)
    
    initials = []
    
    for idx in range(1, num_vehicles + 1):
        initials.append(
            FixedVehicleInitial(
                vehicle_id=idx,
                initial_node=depot_node,  # 모든 차량이 동일한 노드 사용
            )
        )
    
    return initials


# --------------------------------------------------------------------------------------
# 헬퍼 함수
# --------------------------------------------------------------------------------------


def format_stop_sequence(stops: Sequence[Stop]) -> str:
    if not stops:
        return "(경로 없음)"
    return " → ".join(f"{stop.stop_type}:{stop.passenger_id}" for stop in stops)


def node_lonlat(graph: nx.Graph, node: int) -> Tuple[Optional[float], Optional[float]]:
    data = graph.nodes.get(node)
    if not data:
        return None, None
    return data.get("longitude"), data.get("latitude")


def calculate_straight_line_distance(
    graph: nx.Graph,
    node1: int,
    node2: int,
) -> Tuple[float, float]:
    """
    두 노드 간의 직선 거리를 계산합니다 (위경도 기반).
    
    Returns:
        (distance_degrees, distance_km)
        - distance_degrees: 도 단위 거리
        - distance_km: 킬로미터 단위 거리 (Haversine 공식 사용)
    """
    lon1, lat1 = node_lonlat(graph, node1)
    lon2, lat2 = node_lonlat(graph, node2)
    
    if None in (lon1, lat1, lon2, lat2):
        return 0.0, 0.0
    
    # 도 단위 거리
    dx = lon2 - lon1
    dy = lat2 - lat1
    distance_degrees = math.sqrt(dx * dx + dy * dy)
    
    # Haversine 공식을 사용하여 km 단위 거리 계산
    # 지구 반지름 (km)
    R = 6371.0
    
    # 라디안으로 변환
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat_rad = math.radians(lat2 - lat1)
    dlon_rad = math.radians(lon2 - lon1)
    
    # Haversine 공식
    a = math.sin(dlat_rad / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_km = R * c
    
    return distance_degrees, distance_km


def calculate_path_travel_time(
    travel_time_cache: NetworkTravelTimeCache,
    start_node: int,
    path: List[Stop],
    pickup_node: int,
    dropoff_node: int,
) -> Tuple[float, float, float]:
    """
    경로에서 픽업부터 드롭오프까지의 통행시간을 계산합니다.
    
    Args:
        travel_time_cache: 이동 시간 캐시
        start_node: 시작 노드
        path: 경로 (Stop 리스트)
        pickup_node: 픽업 노드
        dropoff_node: 드롭오프 노드
    
    Returns:
        (travel_time, pickup_time, dropoff_time)
        - travel_time: 픽업부터 드롭오프까지의 시간 (초)
        - pickup_time: 시작 노드부터 픽업까지의 시간 (초)
        - dropoff_time: 시작 노드부터 드롭오프까지의 시간 (초)
    """
    current_node = start_node
    pickup_time = 0.0
    dropoff_time = 0.0
    found_pickup = False
    
    for stop in path:
        travel = travel_time_cache.travel_seconds(current_node, stop.node_id)
        if math.isinf(travel):
            return math.inf, math.inf, math.inf
        
        if not found_pickup:
            pickup_time += travel
            if stop.node_id == pickup_node and stop.stop_type == "pickup":
                found_pickup = True
                dropoff_time = pickup_time
        else:
            dropoff_time += travel
            if stop.node_id == dropoff_node and stop.stop_type == "dropoff":
                break
        
        current_node = stop.node_id
    
    travel_time = dropoff_time - pickup_time
    return travel_time, pickup_time, dropoff_time


def build_route_nodes(graph: nx.Graph, start_node: int, stops: Iterable[Stop]) -> List[int]:
    """
    Stop 시퀀스로부터 전체 노드 경로를 구축합니다.
    
    성능 최적화: 시각화 단계에서만 호출되도록 변경됨.
    """
    route: List[int] = [start_node]
    current = start_node
    for stop in stops:
        try:
            segment = nx.shortest_path(graph, current, stop.node_id, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            segment = [current, stop.node_id]
        if route:
            route.extend(segment[1:])
        else:
            route.extend(segment)
        current = stop.node_id
    return route


def route_to_xy(graph: nx.Graph, node_sequence: Iterable[int]) -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    for node in node_sequence:
        lon, lat = node_lonlat(graph, node)
        if lon is None or lat is None:
            continue
        xs.append(lon)
        ys.append(lat)
    return xs, ys


def compute_bounds(points: Iterable[Tuple[Optional[float], Optional[float]]]) -> Tuple[float, float, float, float]:
    xs = [x for x, y in points if x is not None]
    ys = [y for x, y in points if y is not None]
    if not xs or not ys:
        return 0.0, 1.0, 0.0, 1.0
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad_x = max((max_x - min_x) * 0.1, 0.001)
    pad_y = max((max_y - min_y) * 0.1, 0.001)
    return min_x - pad_x, max_x + pad_x, min_y - pad_y, max_y + pad_y


def deduplicate_legend(ax: plt.Axes, fontsize: int = 8) -> None:
    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    unique_handles = []
    unique_labels = []
    for handle, label in zip(handles, labels):
        if label in seen:
            continue
        seen.add(label)
        unique_handles.append(handle)
        unique_labels.append(label)
    if unique_handles:
        ax.legend(unique_handles, unique_labels, loc="best", fontsize=fontsize)


def compute_percentile(data: List[float], p: float) -> float:
    """백분위수 계산 (numpy 없이 기본 라이브러리 활용)"""
    if not data:
        return 0.0
    sorted_data = sorted(data)
    k = (len(sorted_data) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_data[int(k)]
    d0 = sorted_data[int(f)] * (c - k)
    d1 = sorted_data[int(c)] * (k - f)
    return d0 + d1


# --------------------------------------------------------------------------------------
# 동적 차량 이동 함수
# --------------------------------------------------------------------------------------


def advance_vehicle_position(
    vehicle: VehicleState,
    current_time: float,
    travel_time_cache: NetworkTravelTimeCache,
) -> None:
    """
    현재 시각(current_time)을 기준으로 차량의 위치를 동적으로 갱신합니다.

    차량이 현재 경로(path)를 따라 이동한다고 가정하고,
    schedule_start_time 이후 경과한 시간 동안 완료했을 Stop들을 path에서 제거하고
    current_node를 마지막으로 도달한 Stop 노드로 업데이트합니다.

    Args:
        vehicle: 상태를 갱신할 차량 객체 (in-place 수정)
        current_time: 현재 시뮬레이션 시각 (초)
        travel_time_cache: 노드 간 이동 시간 캐시
    """
    if not vehicle.path:
        # 경로가 없으면 차량은 현재 위치에 정차 중 → 갱신 불필요
        return

    # 현재 경로가 할당된 이후 경과한 시간
    elapsed = current_time - vehicle.schedule_start_time
    if elapsed <= 0:
        return

    # current_node에서 시작하여 경과 시간만큼 각 Stop 방문 시도
    remaining_time = elapsed
    new_current_node = vehicle.current_node
    completed_stops = 0

    for stop in vehicle.path:
        travel = travel_time_cache.travel_seconds(new_current_node, stop.node_id)
        if math.isinf(travel):
            # 경로 계산 불가 → 이 Stop 이후는 이동 중단
            break
        if remaining_time >= travel:
            # 이 Stop에 도달 완료
            remaining_time -= travel
            new_current_node = stop.node_id
            completed_stops += 1
        else:
            # 아직 이 Stop에 도달하지 못함 → 이동 중단
            break

    # 완료된 Stop을 path에서 제거하고 current_node 갱신
    if completed_stops > 0:
        vehicle.path = vehicle.path[completed_stops:]
        vehicle.current_node = new_current_node
        # 새 위치부터 새 경로 시작이므로 schedule_start_time 갱신
        vehicle.schedule_start_time = current_time - remaining_time


# --------------------------------------------------------------------------------------
# 시각화 루틴 (최종 결과만 그리기) - 성능 최적화 버전
# --------------------------------------------------------------------------------------


def plot_final_assignment(
    ax: plt.Axes,
    events: Sequence[AssignmentEvent],
    graph: nx.Graph,
    vehicle_initials: List[FixedVehicleInitial],
) -> None:
    """
    모든 배차 이벤트를 하나의 지도에 최종 결과로 표시.
    
    성능 최적화: 경로 노드는 이 함수 내에서만 계산됩니다.
    """
    
    # 모든 관련 노드 수집 (경로 노드 계산 전에 필요한 노드만 수집)
    all_nodes = set()
    for event in events:
        all_nodes.add(event.request.pickup_node)
        all_nodes.add(event.request.dropoff_node)
        all_nodes.add(event.vehicle_start_node)
    
    for initial in vehicle_initials:
        all_nodes.add(initial.initial_node)
    
    # 서브그래프 생성
    subgraph = graph.subgraph(all_nodes)
    
    # 간선 그리기
    for u, v in subgraph.edges():
        x1, y1 = node_lonlat(graph, u)
        x2, y2 = node_lonlat(graph, v)
        if None in (x1, y1, x2, y2):
            continue
        ax.plot([x1, x2], [y1, y2], color="#e0e0e0", linewidth=0.3, zorder=1, alpha=0.5)
    
    # 차량별 색상 매핑
    vehicle_color_map: Dict[int, str] = {}
    fallback_idx = 0
    for event in events:
        if event.vehicle_id not in vehicle_color_map:
            if event.vehicle_id in PRESET_VEHICLE_COLORS:
                vehicle_color_map[event.vehicle_id] = PRESET_VEHICLE_COLORS[event.vehicle_id]
            else:
                fallback_color = FALLBACK_VEHICLE_COLORS[fallback_idx % len(FALLBACK_VEHICLE_COLORS)]
                vehicle_color_map[event.vehicle_id] = fallback_color
                fallback_idx += 1
    
    # 각 차량의 최종 경로 그리기 (시간 순서대로 모든 이벤트 경로 연결)
    vehicle_final_paths: Dict[int, List[int]] = {}
    
    # 차량별로 이벤트를 시간 순서대로 정렬
    vehicle_events: Dict[int, List[AssignmentEvent]] = {}
    for event in events:
        if event.vehicle_id not in vehicle_events:
            vehicle_events[event.vehicle_id] = []
        vehicle_events[event.vehicle_id].append(event)
    
    # 각 차량의 이벤트를 시간 순서대로 정렬
    for vehicle_id in vehicle_events:
        vehicle_events[vehicle_id].sort(key=lambda e: e.request_time)
    
    # 각 차량의 전체 경로 구성 (성능 최적화: 경로 노드는 여기서 계산)
    for vehicle_id, vehicle_event_list in vehicle_events.items():
        # 차량의 초기 위치에서 시작
        initial = next((v for v in vehicle_initials if v.vehicle_id == vehicle_id), None)
        if initial:
            current_node = initial.initial_node
            full_path = [current_node]
        else:
            if vehicle_event_list:
                current_node = vehicle_event_list[0].vehicle_start_node
                full_path = [current_node]
            else:
                full_path = []
        
        # 각 이벤트의 경로를 순차적으로 연결
        for event in vehicle_event_list:
            # 이벤트의 새 경로 노드 계산 (시각화 시에만 수행)
            new_route_nodes = build_route_nodes(graph, event.vehicle_start_node, event.new_path)
            if not new_route_nodes:
                continue
            
            # 현재 노드에서 첫 번째 이벤트 시작 노드까지의 경로가 필요할 수 있음
            if current_node != event.vehicle_start_node:
                # 현재 노드에서 vehicle_start_node까지의 경로 추가
                try:
                    intermediate_path = nx.shortest_path(graph, current_node, event.vehicle_start_node, weight="weight")
                    if len(intermediate_path) > 1:
                        # 첫 번째 노드는 이미 full_path에 있으므로 제외
                        full_path.extend(intermediate_path[1:])
                        current_node = event.vehicle_start_node
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    # 경로를 찾을 수 없으면 vehicle_start_node로 직접 이동
                    if current_node != event.vehicle_start_node:
                        full_path.append(event.vehicle_start_node)
                        current_node = event.vehicle_start_node
            
            # 이벤트의 새 경로 추가 (첫 번째 노드는 이미 포함되어 있을 수 있음)
            if new_route_nodes:
                if full_path and full_path[-1] == new_route_nodes[0]:
                    # 중복 제거
                    full_path.extend(new_route_nodes[1:])
                else:
                    full_path.extend(new_route_nodes)
                
                # 마지막 노드 업데이트
                if new_route_nodes:
                    current_node = new_route_nodes[-1]
        
        vehicle_final_paths[vehicle_id] = full_path
        
        # 전체 경로의 모든 노드를 all_nodes에 추가 (나중에 서브그래프 확장용)
        all_nodes.update(full_path)
    
    # 차량별 최종 경로 그리기
    for vehicle_id, route_nodes in vehicle_final_paths.items():
        color = vehicle_color_map.get(vehicle_id, FALLBACK_VEHICLE_COLORS[0])
        xs, ys = route_to_xy(graph, route_nodes)
        if len(xs) >= 2:
            ax.plot(xs, ys, linestyle="-", color=color, linewidth=2.0,
                   label=f"차량 {vehicle_id} 최종 경로", zorder=3, alpha=0.8)
    
    # 모든 픽업/드롭오프 위치 표시 (요청 번호와 함께)
    # 요청 번호 추출을 위한 매핑 생성
    request_number_map: Dict[str, int] = {}
    for event in events:
        pid = event.request.passenger_id
        if pid not in request_number_map:
            try:
                num = int(pid.split('_')[-1])
            except (ValueError, IndexError):
                num = 0
            request_number_map[pid] = num
    
    # 픽업 위치 (요청 번호와 함께 표시)
    pickup_positions: Dict[int, Tuple[float, float, int]] = {}  # node -> (lon, lat, request_num)
    for event in events:
        node = event.request.pickup_node
        lon, lat = node_lonlat(graph, node)
        if lon is not None and lat is not None:
            request_num = request_number_map.get(event.request.passenger_id, 0)
            pickup_positions[node] = (lon, lat, request_num)
    
    if pickup_positions:
        pickup_xs = [pos[0] for pos in pickup_positions.values()]
        pickup_ys = [pos[1] for pos in pickup_positions.values()]
        ax.scatter(
            pickup_xs, pickup_ys,
            marker="*", s=120, color="#ffb703",
            edgecolor="k", linewidth=0.5,
            label="픽업 위치", zorder=4,
        )
        # 픽업 위치에 번호 표시 (P1, Street_P2, ...)
        for node, (lon, lat, req_num) in pickup_positions.items():
            # 요청 번호 텍스트를 파싱해서 Street인지 Normal인지 확인 (최적화 상 이벤트 리스트 재검색)
            is_street = False
            for e in events:
                if e.request.pickup_node == node and e.request.is_street_hail:
                    is_street = True
                    break
            
            label_text = f"Street_P{req_num}" if is_street else f"P{req_num}"
            ax.text(lon, lat, label_text, fontsize=9, ha="left", va="bottom",
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8, edgecolor="#ffb703"),
                   zorder=5)
    
    # 드롭오프 위치 (요청 번호와 함께 표시)
    dropoff_positions: Dict[int, Tuple[float, float, int]] = {}  # node -> (lon, lat, request_num)
    for event in events:
        node = event.request.dropoff_node
        lon, lat = node_lonlat(graph, node)
        if lon is not None and lat is not None:
            request_num = request_number_map.get(event.request.passenger_id, 0)
            dropoff_positions[node] = (lon, lat, request_num)
    
    if dropoff_positions:
        dropoff_xs = [pos[0] for pos in dropoff_positions.values()]
        dropoff_ys = [pos[1] for pos in dropoff_positions.values()]
        ax.scatter(
            dropoff_xs, dropoff_ys,
            marker="X", s=100, color="#d62828",
            linewidth=0.8, label="드롭오프 위치", zorder=4,
        )
        # 드롭오프 위치에 번호 표시 (D1, Street_D2, ...)
        for node, (lon, lat, req_num) in dropoff_positions.items():
            is_street = False
            for e in events:
                if e.request.dropoff_node == node and e.request.is_street_hail:
                    is_street = True
                    break
                    
            label_text = f"Street_D{req_num}" if is_street else f"D{req_num}"
            ax.text(lon, lat, label_text, fontsize=9, ha="left", va="bottom",
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8, edgecolor="#d62828"),
                   zorder=5)
    
    # 차량 초기 위치
    initial_xs, initial_ys = [], []
    for initial in vehicle_initials:
        lon, lat = node_lonlat(graph, initial.initial_node)
        if lon is not None and lat is not None:
            initial_xs.append(lon)
            initial_ys.append(lat)
    
    if initial_xs:
        ax.scatter(
            initial_xs, initial_ys,
            marker="o", s=80, color="#2a9d8f",
            edgecolor="white", linewidth=0.6,
            label="차량 초기 위치", zorder=4,
        )
    
    # 범위 설정
    bound_points: List[Tuple[Optional[float], Optional[float]]] = []
    for event in events:
        bound_points.append(node_lonlat(graph, event.request.pickup_node))
        bound_points.append(node_lonlat(graph, event.request.dropoff_node))
    for initial in vehicle_initials:
        bound_points.append(node_lonlat(graph, initial.initial_node))
    
    min_x, max_x, min_y, max_y = compute_bounds(bound_points)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", linewidth=0.4, alpha=0.4)
    ax.set_xlabel("경도", fontsize=10)
    ax.set_ylabel("위도", fontsize=10)
    ax.tick_params(labelsize=8)
    ax.set_title(f"화성똑버스03 DRT 배정 최종 결과 (총 {len(events)}개 요청)", fontsize=12)
    
    deduplicate_legend(ax, fontsize=9)


def visualize_final_result(
    events: Sequence[AssignmentEvent],
    graph: nx.Graph,
    vehicle_initials: List[FixedVehicleInitial],
    save_path: Optional[str] = None,
) -> None:
    """최종 배정 결과를 하나의 지도로 시각화."""
    if not events:
        print("시각화할 배차 이벤트가 없습니다.")
        return
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    plot_final_assignment(ax, events, graph, vehicle_initials)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=200)
        print(f"[저장 완료] {save_path}")
    else:
        plt.show()


# --------------------------------------------------------------------------------------
# 시뮬레이션 실행 클래스 (성능 최적화 버전)
# --------------------------------------------------------------------------------------


class RealizedDRTSimulation:
    """향남신도시 지역으로 제한된 DRT 배정 시뮬레이션 (화성똑버스03 재현)."""

    def __init__(
        self,
        num_vehicles: int = NUM_VEHICLES,
        vehicle_capacity: int = VEHICLE_CAPACITY,
        demand_seed: int = DEMAND_SEED,
        street_hail_demand_seed: int = STREET_HAIL_DEMAND_SEED,
        vehicle_seed: int = VEHICLE_SEED,
        min_lon: float = REGION_MIN_LON,
        max_lon: float = REGION_MAX_LON,
        min_lat: float = REGION_MIN_LAT,
        max_lat: float = REGION_MAX_LAT,
    ) -> None:
        self.graph = load_network_graph()
        self.engine = DRTAssignmentEngine(
            self.graph,
            max_path_length=MAX_PATH_LENGTH,  # 배정 코드의 상수를 직접 참조
            max_dispatch_eta=MAX_DISPATCH_ETA_SECONDS,  # 배정 코드의 상수를 직접 참조
            street_hail_travel_time_increase_limit=STREET_HAIL_TRAVEL_TIME_INCREASE_LIMIT
        )
        self.num_vehicles = num_vehicles
        self.vehicle_capacity = vehicle_capacity
        
        # 지역 필터링: 지정된 좌표 범위 내의 노드만 사용
        print(f"[지역 필터링] 향남신도시 지역으로 제한")
        print(f"  경도 범위: {min_lon:.6f} ~ {max_lon:.6f}")
        print(f"  위도 범위: {min_lat:.6f} ~ {max_lat:.6f}")
        
        self.allowed_nodes = filter_nodes_by_region(
            self.graph, min_lon, max_lon, min_lat, max_lat
        )
        
        if not self.allowed_nodes:
            raise ValueError(
                f"지정된 좌표 범위 내에 노드가 없습니다. "
                f"경도: {min_lon}~{max_lon}, 위도: {min_lat}~{max_lat}"
            )
        
        print(f"  ✅ 필터링된 노드 수: {len(self.allowed_nodes)}개 (전체 {len(self.graph.nodes)}개 중)")
        print()
        
        # 고정된 차량 초기 위치 생성 (필터링된 노드만 사용)
        self.vehicle_initials = generate_fixed_vehicle_initials(
            num_vehicles, self.graph, vehicle_seed, allowed_nodes=self.allowed_nodes
        )
        
        # 차량 상태 초기화
        self.vehicles: List[VehicleState] = []
        for initial in self.vehicle_initials:
            vehicle = VehicleState(
                vehicle_id=initial.vehicle_id,
                current_node=initial.initial_node,
                capacity=vehicle_capacity,
                onboard_passengers=0,
            )
            self.vehicles.append(vehicle)
        
        print(f"[초기화 완료] 차량 {num_vehicles}대, 용량 {vehicle_capacity}명")
        print(f"[차량 초기 위치 (차고지)]")
        # 모든 차량이 동일한 차고지를 사용하므로 첫 번째 차량만 출력
        if self.vehicle_initials:
            depot_node = self.vehicle_initials[0].initial_node
            lon, lat = node_lonlat(self.graph, depot_node)
            print(f"  차고지 노드: {depot_node} (경도: {lon:.6f}, 위도: {lat:.6f})")
            print(f"  모든 차량({num_vehicles}대)이 동일한 차고지에서 출발합니다.")
        print()

    def process_request(self, request: PassengerRequest, current_time: float) -> Optional[AssignmentEvent]:
        """
        단일 요청을 처리하고 배차 이벤트를 반환.
        
        성능 최적화: 경로 노드 구축을 제거하여 배정 단계의 계산량을 대폭 감소시킴.
        경로 노드는 시각화 단계에서만 계산됩니다.
        """
        # 배정 처리 (current_time 넘김)
        if request.is_street_hail:
            assigned_vehicle, new_path, cost = self.engine.assign_street_hail_request(self.vehicles, request, current_time)
        else:
            assigned_vehicle, new_path, cost = self.engine.assign_request(self.vehicles, request, current_time)
        
        # 배정 완료 시각 (실제 시뮬레이션 current_time)
        assignment_time = current_time

        if assigned_vehicle is None:
            return None

        previous_path = assigned_vehicle.clone_path()
        start_node = assigned_vehicle.current_node

        # 통행시간 계산 (픽업부터 드롭오프까지)
        travel_time, pickup_time_from_start, dropoff_time_from_start = calculate_path_travel_time(
            self.engine.travel_time_cache,
            start_node,
            new_path,
            request.pickup_node,
            request.dropoff_node,
        )

        # 픽업 및 드롭오프 시각 계산 (배정 시점의 current_time 기준이 아니라 차량의 스케줄 기준)
        pickup_time = assignment_time + pickup_time_from_start
        dropoff_time = assignment_time + dropoff_time_from_start

        # 대기시간 등 5개 주요 지표 계산
        # request.request_time은 실제로 승객이 요청한 과거 시각일 수 있음
        waiting_time = pickup_time - request.request_time

        original_path_time = self.engine._calculate_path_time(start_node, previous_path)
        new_path_time = self.engine._calculate_path_time(start_node, new_path)
        if math.isinf(original_path_time):
            original_path_time = 0.0

        cost_increase = max(0.0, new_path_time - original_path_time)
        wait_assign = max(0.0, assignment_time - request.request_time)
        wait_pickup = max(0.0, pickup_time - assignment_time)
        total_wait = wait_assign + wait_pickup
        
        # 직선 거리 계산 (도 단위와 km 단위)
        straight_line_distance_degrees, straight_line_distance_km = calculate_straight_line_distance(
            self.graph,
            request.pickup_node,
            request.dropoff_node,
        )
        
        # 평균 속도 계산 (km/h)
        # travel_time은 초 단위이므로, km/h = (거리_km / 시간_초) * 3600
        speed_kmh = (straight_line_distance_km / travel_time * 3600) if travel_time > 0 else 0.0
        
        event = AssignmentEvent(
            request=request,
            vehicle_id=assigned_vehicle.vehicle_id,
            vehicle_start_node=start_node,
            cost=cost,
            cost_increase=cost_increase,
            new_path_time=new_path_time,
            total_wait=total_wait,
            wait_assign=wait_assign,
            wait_pickup=wait_pickup,
            previous_path=previous_path,
            new_path=list(new_path),
            request_time=request.request_time,  # 승객이 큐에 들어온 실제 시간
            assignment_time=assignment_time,
            waiting_time=waiting_time,
            travel_time=travel_time,
            pickup_time=pickup_time,
            dropoff_time=dropoff_time,
            straight_line_distance_degrees=straight_line_distance_degrees,
            straight_line_distance_km=straight_line_distance_km,
            speed_kmh=speed_kmh,
        )
        
        assigned_vehicle.path = new_path
        assigned_vehicle.schedule_start_time = current_time  # [동적이동] 새 경로 할당 시각 기록
        
        # 터미널 출력 (좌표값 및 시간 정보 포함)
        pickup_lon, pickup_lat = node_lonlat(self.graph, request.pickup_node)
        dropoff_lon, dropoff_lat = node_lonlat(self.graph, request.dropoff_node)

        if request.is_street_hail:
            print(f"✋ [요청 {request.passenger_id}] 차량 {assigned_vehicle.vehicle_id} 길거리 픽업 성공 (비용: {cost:,.2f}, 요청 시각: {request.request_time:.1f}초, 배정 시각: {assignment_time:.1f}초)")
        else:
            print(f"[요청 {request.passenger_id}] 차량 {assigned_vehicle.vehicle_id} 일반 배정 (비용: {cost:,.2f}, 요청 시각: {request.request_time:.1f}초, 배정 시각: {assignment_time:.1f}초)")

        print(f"  - 픽업 노드: {request.pickup_node} (경도: {pickup_lon:.6f}, 위도: {pickup_lat:.6f})")
        print(f"  - 드롭오프 노드: {request.dropoff_node} (경도: {dropoff_lon:.6f}, 위도: {dropoff_lat:.6f})")
        print(f"  - 기존 경로: {format_stop_sequence(previous_path)}")
        print(f"  - 신규 경로: {format_stop_sequence(new_path)}")
        print(f"  ⏱️  시간 정보:")
        print(f"     • 대기시간 (총 대기): {total_wait:.2f}초 (배정 대기: {wait_assign:.2f}초, 픽업 대기: {wait_pickup:.2f}초)")
        print(f"     • 통행시간 (픽업→드롭오프): {travel_time:.2f}초 ({travel_time/60:.2f}분)")
        if request.is_street_hail:
            print(f"     • ⚠️ 기존 노선 지연 발생: {cost_increase:.2f}초 (허용치 {self.engine.street_hail_travel_time_increase_limit:.0f}초 이내 만족)")
        print(f"     • 픽업 예상 시각: {pickup_time:.1f}초 ({pickup_time/60:.1f}분)")
        print(f"     • 드롭오프 예상 시각: {dropoff_time:.1f}초 ({dropoff_time/60:.1f}분)")
        print(f"  📏 거리 정보:")
        print(f"     • 직선 거리: {straight_line_distance_km:.2f}km ({straight_line_distance_degrees:.6f}도)")
        print(f"     • 평균 속도: {speed_kmh:.2f}km/h")
        print()
        
        return event

    def run_simulation(
        self,
        fixed_demands: List[FixedDemand],
    ) -> Tuple[List[AssignmentEvent], float, List[PassengerRequest]]:
        """
        고정된 전체 디멘드 목록을 사용하여 시뮬레이션을 실행합니다.
        
        전체 디멘드는 일반과 길거리가 섞여 있으며, 각자의 request_interval 기준으로 틱(tick)에 도달하면 큐에 들어옵니다.
        """
        events: List[AssignmentEvent] = []
        pending_requests: List[PassengerRequest] = []
        failed_requests: List[PassengerRequest] = [] # 추가: 실패한 요청 목록
        current_time = 0.0
        max_simulation_time = 3600.0 * 2  # 안전 장치: 최대 2시간
        
        print(f"[시뮬레이션 시작] 총 {len(fixed_demands)}개 혼합 디멘드 (큐 기반 재배정 포함)\n")
        
        start_time = time.perf_counter()
        
        # arrival_time 기준으로 미리 스케줄링하기 위해 변환
        demand_schedule = []
        
        normal_arrival = 0.0
        street_arrival = 0.0
        
        for d in fixed_demands:
            if d.is_street_hail:
                street_arrival += d.request_interval
                demand_schedule.append((street_arrival, d))
            else:
                normal_arrival += d.request_interval
                demand_schedule.append((normal_arrival, d))
                
        # 시간순 정렬
        demand_schedule.sort(key=lambda x: x[0])
        
        schedule_idx = 0
        total_demands = len(demand_schedule)
        
        # 제일 작은 tick을 찾아서 최소 빈도 설정 (통상 30초, 60초면 최소공배수나 GCD)
        tick_interval = 10.0 # 10초마다 체크하도록 해상도 높임
        
        # 요청 발생 및 처리 루프
        while (schedule_idx < total_demands or pending_requests) and current_time < max_simulation_time:
            # 0. [동적이동] 매 틱마다 모든 차량의 위치를 현재 시각 기준으로 갱신
            for vehicle in self.vehicles:
                advance_vehicle_position(vehicle, current_time, self.engine.travel_time_cache)

            # 1. 이번 틱(current_time) 이하에 도착한 요청을 큐에 모두 추가
            while schedule_idx < total_demands and demand_schedule[schedule_idx][0] <= current_time:
                sched_time, new_demand = demand_schedule[schedule_idx]
                req = PassengerRequest(
                    passenger_id=new_demand.passenger_id,
                    pickup_node=new_demand.pickup_node,
                    dropoff_node=new_demand.dropoff_node,
                    request_time=sched_time,  # 요청 시각
                    is_street_hail=new_demand.is_street_hail
                )
                pending_requests.append(req)
                schedule_idx += 1
                
            # 2. 대기 큐(`pending_requests`)에 있는 요청들 배정 재시도
            next_pending = []
            for req in pending_requests:
                event = self.process_request(req, current_time)
                
                if event:
                    events.append(event)
                else:
                    # 배정 실패 시 대기시간 초과 여부 확인
                    elapsed = current_time - req.request_time
                    if req.is_street_hail and elapsed > STREET_HAIL_QUEUE_TIMEOUT_SECONDS:
                        print(f"💨 [요청 {req.passenger_id}] {STREET_HAIL_QUEUE_TIMEOUT_SECONDS/60:.1f}분 초과 길거리 탑승 실패 (폐기됨 - 사유: {req.street_fail_reason})")
                        failed_requests.append(req)
                    elif not req.is_street_hail and elapsed > MAX_AWAIT_TIME_SECONDS:
                        print(f"❌ [요청 {req.passenger_id}] {MAX_AWAIT_TIME_SECONDS/60:.1f}분 초과 일반 배차 실패 (폐기됨)")
                        failed_requests.append(req)
                    else:
                        next_pending.append(req)
                        
            pending_requests = next_pending
            
            # 3. 시간 진행
            current_time += tick_interval
        
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        remain = len(pending_requests)
        if remain > 0:
            print(f"⚠️ 시뮬레이션 종료 후에도 배정되지 못한 요청 수: {remain}개")
            for r in pending_requests:
                print(f"  - 요청 {r.passenger_id} (요청 시간: {r.request_time}초, Street: {r.is_street_hail})")
                failed_requests.append(r)
            
        print(f"[시뮬레이션 완료] 총 {len(events)}개 요청 배정 성공 / 누적 시뮬레이션 시간: {current_time}초\n")
        return events, elapsed_time, failed_requests


# --------------------------------------------------------------------------------------
# 메인 실행
# --------------------------------------------------------------------------------------


def main() -> None:
    """메인 실행 함수."""
    print("=" * 70)
    print("화성똑버스03 DRT 네트워크 배정 시뮬레이션 (Realized 버전)")
    print("향남신도시 지역으로 제한된 시뮬레이션")
    print("=" * 70)
    print()
    
    # ================================================================================
    # 시뮬레이션 설정 확인
    # ================================================================================
    print(f"[설정] 디멘드 개수: {NUM_DEMANDS}개")
    print(f"[설정] 요청 간격: {REQUEST_INTERVAL_SECONDS}초")
    print(f"[설정] 차량 수: {NUM_VEHICLES}대")
    print(f"[설정] 차량 용량: {VEHICLE_CAPACITY}명")
    print()
    # ================================================================================
    
    # 시뮬레이션 초기화 (지역 필터링 포함)
    simulation = RealizedDRTSimulation(
        num_vehicles=NUM_VEHICLES,
        vehicle_capacity=VEHICLE_CAPACITY,
    )
    
    # 고정된 일반/길거리 디멘드 생성
    fixed_demands_normal = generate_fixed_demands(
        simulation.graph, NUM_DEMANDS, DEMAND_SEED, allowed_nodes=simulation.allowed_nodes
    )
    fixed_demands_street = generate_street_hail_demands(
        simulation.graph, NUM_STREET_HAIL_DEMANDS, STREET_HAIL_DEMAND_SEED, allowed_nodes=simulation.allowed_nodes
    )
    
    # 두 디멘드를 한 리스트에 합침 (run_simulation 내부에서 시간표 스케줄링함)
    all_demands = fixed_demands_normal + fixed_demands_street
    print(f"[디멘드 생성 완료] 일반 {NUM_DEMANDS}개, 길거리 {NUM_STREET_HAIL_DEMANDS}개, 총 {len(all_demands)}개 디멘드 생성\n")
    
    # 시뮬레이션 실행 (시뮬레이션 상으로는 10초 간격 틱)
    events, simulation_time, failed_requests = simulation.run_simulation(
        all_demands,
    )
    
    # 성능 측정 결과 출력
    print("=" * 70)
    print("📊 성능 측정 결과")
    print("=" * 70)
    print(f"총 디멘드 수: {len(all_demands)}개 (일반 {NUM_DEMANDS}개, 길거리 {NUM_STREET_HAIL_DEMANDS}개)")
    
    normal_events = [e for e in events if not e.request.is_street_hail]
    street_events = [e for e in events if e.request.is_street_hail]
    
    print(f"전체 배정 성공: {len(events)}개")
    print(f"  - 일반 승객 배정 성공: {len(normal_events)} / {NUM_DEMANDS}")
    print(f"  - 길거리 승객 배정 성공: {len(street_events)} / {NUM_STREET_HAIL_DEMANDS}")
    
    # 실패 원인 상세 분석 (시뮬레이션 종료 후 pending에 남아있는 요청들 기준)
    print(f"전체 배정 실패: {len(all_demands) - len(events)}개")
    if len(failed_requests) > 0:
        fail_normal_timeout = 0
        fail_street_no_vehicle = 0
        fail_street_exceed_limit = 0
        
        for req in failed_requests:
            if req.is_street_hail:
                if req.street_fail_reason == "과중한_경로_증가":
                    fail_street_exceed_limit += 1
                else:
                    fail_street_no_vehicle += 1
            else:
                fail_normal_timeout += 1

        print(f"  - [일반] 대기시간 초과로 인한 실패: {fail_normal_timeout}명")
        print(f"  - [길거리] 길거리 대기시간 초과로 인한 실패 (경로상 차량 부재): {fail_street_no_vehicle}명")
        print(f"  - [길거리] 길거리 탑승 시도했지만, 과중한 차량 경로 증가로 인한 배정 실패: {fail_street_exceed_limit}명")
        
    print(f"배정 실행 시간: {simulation_time:.3f}초 ({simulation_time * 1000:.2f}ms)")
    if len(events) > 0:
        print(f"요청당 평균 시간: {simulation_time / len(events):.3f}초 ({simulation_time * 1000 / len(events):.2f}ms)")
    print("=" * 70)
    print()
    
    # 시간 통계 출력
    if events:
        print("=" * 70)
        print("⏱️  승객별 시간 통계")
        print("=" * 70)
        
        # 요청 번호 매핑 생성 (P1, D1 등) - 모든 출력 블록에서 일관되게 사용
        request_number_map: Dict[str, int] = {}
        for event in events:
            pid = event.request.passenger_id
            if pid not in request_number_map:
                try:
                    num = int(pid.split('_')[-1])
                except (ValueError, IndexError):
                    num = 0
                request_number_map[pid] = num
        
        waiting_times = [e.waiting_time for e in events]
        travel_times = [e.travel_time for e in events if not math.isinf(e.travel_time)]
        distances_km = [e.straight_line_distance_km for e in events if e.straight_line_distance_km > 0]
        
        print(f"\n📋 대기시간 (요청부터 픽업까지 총 대기):")
        if waiting_times:
            print(f"   평균: {sum(waiting_times)/len(waiting_times):.2f}초")
            print(f"   최소: {min(waiting_times):.2f}초")
            print(f"   최대: {max(waiting_times):.2f}초")
        
        print(f"\n🚗 통행시간 (픽업→드롭오프):")
        if travel_times:
            avg_travel = sum(travel_times) / len(travel_times)
            print(f"   평균: {avg_travel:.2f}초 ({avg_travel/60:.2f}분)")
            print(f"   최소: {min(travel_times):.2f}초 ({min(travel_times)/60:.2f}분)")
            print(f"   최대: {max(travel_times):.2f}초 ({max(travel_times)/60:.2f}분)")
        
        print(f"\n📏 직선 거리:")
        distances_km = [e.straight_line_distance_km for e in events if e.straight_line_distance_km > 0]
        if distances_km:
            avg_dist = sum(distances_km) / len(distances_km)
            print(f"   평균: {avg_dist:.2f}km")
            print(f"   최소: {min(distances_km):.2f}km")
            print(f"   최대: {max(distances_km):.2f}km")
        
        print(f"\n📊 평균 속도 (km/h):")
        speeds_kmh = [e.speed_kmh for e in events if e.speed_kmh > 0 and not math.isinf(e.speed_kmh)]
        if speeds_kmh:
            avg_speed = sum(speeds_kmh) / len(speeds_kmh)
            print(f"   평균: {avg_speed:.2f}km/h")
            print(f"   최소: {min(speeds_kmh):.2f}km/h")
            print(f"   최대: {max(speeds_kmh):.2f}km/h")
        
        # 5개 핵심 지표 통계 테이블 출력
        print("\n" + "=" * 70)
        print("📊 5개 핵심 지표 통계 (mean / p95 / p99 / max)")
        print("=" * 70)
        print(f"| {'지표 (Metrics)':<15} | {'Mean':>10} | {'p95':>10} | {'p99':>10} | {'Max':>10} |")
        print(f"|{'-'*17}|{'-'*12}|{'-'*12}|{'-'*12}|{'-'*12}|")
        
        metrics_dict = {
            "cost_increase": [e.cost_increase for e in events],
            "new_path_time": [e.new_path_time for e in events if not math.isinf(e.new_path_time)],
            "total_wait": [e.total_wait for e in events],
            "wait_assign": [e.wait_assign for e in events],
            "wait_pickup": [e.wait_pickup for e in events]
        }
        
        for name, data in metrics_dict.items():
            if data:
                mean_v = sum(data) / len(data)
                p95_v = compute_percentile(data, 95)
                p99_v = compute_percentile(data, 99)
                max_v = max(data)
                print(f"| {name:<15} | {mean_v:10.2f} | {p95_v:10.2f} | {p99_v:10.2f} | {max_v:10.2f} |")
            else:
                print(f"| {name:<15} | {'-':>10} | {'-':>10} | {'-':>10} | {'-':>10} |")
        
        print("\n" + "=" * 70)
        print("📋 개별 요청 상세 정보")
        print("=" * 70)
        for idx, event in enumerate(events, 1):
            pid_num = request_number_map.get(event.request.passenger_id, idx)
            print(f"\n[{pid_num}] 요청 {event.request.passenger_id}:")
            print(f"   차량: {event.vehicle_id}")
            print(f"   대기시간: {event.waiting_time:.2f}초")
            if not math.isinf(event.travel_time):
                print(f"   통행시간: {event.travel_time:.2f}초 ({event.travel_time/60:.2f}분)")
                print(f"   직선 거리: {event.straight_line_distance_km:.2f}km ({event.straight_line_distance_degrees:.6f}도)")
                if event.speed_kmh > 0:
                    print(f"   평균 속도: {event.speed_kmh:.2f}km/h")
            else:
                print(f"   통행시간: 계산 불가 (경로 없음)")
        print("=" * 70)
        print()
    
    # 차량별 이동경로 출력
    if events:
        print("=" * 70)
        print("🚗 차량별 이동경로 (실제 경로 순서)")
        print("=" * 70)
        
        # 차량별로 이벤트 그룹화 및 시간 순서대로 정렬
        vehicle_events: Dict[int, List[AssignmentEvent]] = {}
        for event in events:
            if event.vehicle_id not in vehicle_events:
                vehicle_events[event.vehicle_id] = []
            vehicle_events[event.vehicle_id].append(event)
        
        # 각 차량의 이벤트를 시간 순서대로 정렬
        for vehicle_id in vehicle_events:
            vehicle_events[vehicle_id].sort(key=lambda e: e.request_time)
        
        # 차량별 경로 출력 (실제 경로 순서 반영)
        for vehicle_id in sorted(vehicle_events.keys()):
            vehicle_event_list = vehicle_events[vehicle_id]
            
            # 차량의 초기 위치(차고지) 찾기
            initial = next((v for v in simulation.vehicle_initials if v.vehicle_id == vehicle_id), None)
            if not initial:
                continue
            
            depot_node = initial.initial_node
            
            # 실제 경로 구성: 마지막 이벤트의 new_path가 최종 경로를 포함
            # 마지막 이벤트의 경로가 모든 이전 경로를 포함하므로 이를 사용
            if not vehicle_event_list:
                continue
            
            # 마지막 이벤트의 경로가 최종 경로
            final_event = vehicle_event_list[-1]
            final_path = final_event.new_path
            
            # 경로 구성: 차고지 -> 실제 경로 순서대로
            route_parts = []
            
            # 차고지 추가 (00:00:00)
            route_parts.append(f"차고지(00:00:00)")
            
            # Stop별 시간 계산을 위한 매핑
            stop_times: Dict[Tuple[str, str], float] = {}  # (stop_type, passenger_id) -> time
            
            # 각 이벤트의 픽업/드롭오프 시간을 기록
            for event in vehicle_event_list:
                req_id = event.request.passenger_id
                stop_times[("pickup", req_id)] = event.pickup_time
                stop_times[("dropoff", req_id)] = event.dropoff_time
            
            # Stop과 시간 정보를 함께 저장하고 시간 순서대로 정렬
            stops_with_time: List[Tuple[Stop, float, int]] = []  # (stop, time, req_num)
            for stop in final_path:
                req_num = request_number_map.get(stop.passenger_id, 0)
                stop_time = stop_times.get((stop.stop_type, stop.passenger_id), 0.0)
                stops_with_time.append((stop, stop_time, req_num))
            
            # 시간 순서대로 정렬
            stops_with_time.sort(key=lambda x: x[1])  # stop_time 기준으로 정렬
            
            # 시간 순서대로 출력
            for stop, stop_time, req_num in stops_with_time:
                # 시간 (시간:분:초 형식)
                time_hours = int(stop_time // 3600)
                time_minutes = int((stop_time % 3600) // 60)
                time_seconds = int(stop_time % 60)
                
                prefix = "Street_" if stop.is_street_hail else ""
                
                if stop.stop_type == "pickup":
                    route_parts.append(f"{prefix}P{req_num}({time_hours:02d}:{time_minutes:02d}:{time_seconds:02d})")
                else:
                    route_parts.append(f"{prefix}D{req_num}({time_hours:02d}:{time_minutes:02d}:{time_seconds:02d})")
            
            # 경로 출력
            route_str = " -> ".join(route_parts)
            print(f"\n차량 {vehicle_id}:")
            print(f"  {route_str}")
        
        # [참고] 메시지는 모든 차량 경로 출력 후 한 번만 출력
        print(f"\n  [참고] 이 경로는 모든 요청이 배정된 후의 최종 경로입니다.")
        print(f"  [참고] 각 요청 배정 시 출력된 '신규 경로'는 그 시점의 경로이며, 이후 다른 요청이 배정되면서 변경될 수 있습니다.")
        print("\n" + "=" * 70)
        print()
    
    # 최종 결과 시각화
    if events:
        print("[시각화 시작] 최종 결과 지도 생성 중...\n")
        visualize_final_result(
            events,
            simulation.graph,
            simulation.vehicle_initials,
            save_path=None,  # None이면 화면에 표시, 경로를 지정하면 저장
        )
    else:
        print("시각화할 배차 이벤트가 없습니다.")


if __name__ == "__main__":
    main()
