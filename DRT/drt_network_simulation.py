"""
고정된 디멘드와 차량 초기 위치를 사용하는 DRT 배정 시뮬레이션 및 시각화 모듈.

`drt_network_assignment.py`의 배차 로직과 `drt_network_visualization.py`의 시각화 기능을 통합하여,
미리 설정된 디멘드(요청/도착)와 차량 초기 위치를 사용하여 시뮬레이션을 실행합니다.

주요 특징:
    - 디멘드와 차량 초기 위치를 고정하여 재현 가능한 실험
    - 디멘드 100개까지 대응 가능
    - 각 디멘드는 5초 간격으로 요청이 들어옴
    - 최종 지도만 시각화 (중간 과정은 터미널 출력)
    - 좌표값은 지도에 표시하지 않고 터미널에만 출력
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib import font_manager
import networkx as nx

from drt_network_assignment import (
    DRTAssignmentEngine,
    PassengerRequest,
    Stop,
    VehicleState,
    load_network_graph,
    select_random_node,
)

# --------------------------------------------------------------------------------------
# ================================================================================
# 디멘드 개수 설정 (여기서 변경하세요!)
# ================================================================================
NUM_DEMANDS = 20  # 디멘드 개수 (최대 100개까지 설정 가능)
# ================================================================================
# --------------------------------------------------------------------------------------

# 시뮬레이션 설정
REQUEST_INTERVAL_SECONDS = 5  # 각 디멘드 요청 간격 (초)
NUM_VEHICLES = 2  # 차량 수
VEHICLE_CAPACITY = 4  # 차량 용량

# 시드 값 (고정된 디멘드 생성을 위해)
DEMAND_SEED = 42
VEHICLE_SEED = 123

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
    """단일 배차 이벤트 기록."""

    request: PassengerRequest
    vehicle_id: int
    vehicle_start_node: int
    cost: float
    previous_path: List[Stop]
    new_path: List[Stop]
    previous_route_nodes: List[int]
    new_route_nodes: List[int]
    request_time: float  # 요청이 들어온 시각 (초)


@dataclass
class FixedDemand:
    """고정된 디멘드 정보."""

    passenger_id: str
    pickup_node: int
    dropoff_node: int


@dataclass
class FixedVehicleInitial:
    """고정된 차량 초기 위치."""

    vehicle_id: int
    initial_node: int


# --------------------------------------------------------------------------------------
# 고정된 디멘드 및 차량 초기 위치 생성
# --------------------------------------------------------------------------------------


def generate_fixed_demands(graph: nx.Graph, num_demands: int, seed: int) -> List[FixedDemand]:
    """
    고정된 디멘드 목록을 생성합니다.
    
    Args:
        graph: 네트워크 그래프
        num_demands: 생성할 디멘드 개수 (최대 100)
        seed: 랜덤 시드
    
    Returns:
        고정된 디멘드 목록
    """
    if num_demands > 100:
        print(f"⚠️  디멘드 개수가 100을 초과합니다. 100개로 제한합니다.")
        num_demands = 100
    
    random_gen = random.Random(seed)
    nodes = list(graph.nodes)
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
            )
        )
    
    return demands


def generate_fixed_vehicle_initials(num_vehicles: int, graph: nx.Graph, seed: int) -> List[FixedVehicleInitial]:
    """
    고정된 차량 초기 위치를 생성합니다.
    
    Args:
        num_vehicles: 차량 수
        graph: 네트워크 그래프
        seed: 랜덤 시드
    
    Returns:
        고정된 차량 초기 위치 목록
    """
    random_gen = random.Random(seed)
    nodes = list(graph.nodes)
    initials = []
    
    for idx in range(1, num_vehicles + 1):
        initial_node = random_gen.choice(nodes)
        initials.append(
            FixedVehicleInitial(
                vehicle_id=idx,
                initial_node=initial_node,
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


def build_route_nodes(graph: nx.Graph, start_node: int, stops: Iterable[Stop]) -> List[int]:
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
    ys = [y for x, y in points if x is not None]
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


# --------------------------------------------------------------------------------------
# 시각화 루틴 (최종 결과만 그리기)
# --------------------------------------------------------------------------------------


def plot_final_assignment(
    ax: plt.Axes,
    events: Sequence[AssignmentEvent],
    graph: nx.Graph,
    vehicle_initials: List[FixedVehicleInitial],
) -> None:
    """모든 배차 이벤트를 하나의 지도에 최종 결과로 표시."""
    
    # 모든 관련 노드 수집
    all_nodes = set()
    for event in events:
        all_nodes.update(event.previous_route_nodes)
        all_nodes.update(event.new_route_nodes)
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
    
    # 각 차량의 전체 경로 구성
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
            if not event.new_route_nodes:
                continue
            
            # 현재 노드에서 첫 번째 이벤트 시작 노드까지의 경로가 필요할 수 있음
            # 하지만 new_route_nodes는 이미 vehicle_start_node에서 시작하므로
            # 현재 노드가 vehicle_start_node와 다르면 경로를 연결해야 함
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
            if event.new_route_nodes:
                if full_path and full_path[-1] == event.new_route_nodes[0]:
                    # 중복 제거
                    full_path.extend(event.new_route_nodes[1:])
                else:
                    full_path.extend(event.new_route_nodes)
                
                # 마지막 노드 업데이트
                if event.new_route_nodes:
                    current_node = event.new_route_nodes[-1]
        
        vehicle_final_paths[vehicle_id] = full_path
    
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
    for idx, event in enumerate(events, 1):
        if event.request.passenger_id not in request_number_map:
            request_number_map[event.request.passenger_id] = idx
    
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
        # 픽업 위치에 번호 표시 (P1, P2, ...)
        for node, (lon, lat, req_num) in pickup_positions.items():
            ax.text(lon, lat, f"P{req_num}", fontsize=9, ha="left", va="bottom",
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
        # 드롭오프 위치에 번호 표시 (D1, D2, ...)
        for node, (lon, lat, req_num) in dropoff_positions.items():
            ax.text(lon, lat, f"D{req_num}", fontsize=9, ha="left", va="bottom",
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
    ax.set_title(f"DRT 배정 최종 결과 (총 {len(events)}개 요청)", fontsize=12)
    
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
# 시뮬레이션 실행 클래스
# --------------------------------------------------------------------------------------


class FixedDemandSimulation:
    """고정된 디멘드를 사용하는 DRT 배정 시뮬레이션."""

    def __init__(
        self,
        num_vehicles: int = NUM_VEHICLES,
        vehicle_capacity: int = VEHICLE_CAPACITY,
        demand_seed: int = DEMAND_SEED,
        vehicle_seed: int = VEHICLE_SEED,
    ) -> None:
        self.graph = load_network_graph()
        self.engine = DRTAssignmentEngine(self.graph)
        self.num_vehicles = num_vehicles
        self.vehicle_capacity = vehicle_capacity
        
        # 고정된 차량 초기 위치 생성
        self.vehicle_initials = generate_fixed_vehicle_initials(num_vehicles, self.graph, vehicle_seed)
        
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
        print(f"[차량 초기 위치]")
        for initial in self.vehicle_initials:
            lon, lat = node_lonlat(self.graph, initial.initial_node)
            print(f"  차량 {initial.vehicle_id}: 노드 {initial.initial_node} (경도: {lon}, 위도: {lat})")
        print()

    def process_request(self, request: PassengerRequest, request_time: float) -> Optional[AssignmentEvent]:
        """단일 요청을 처리하고 배차 이벤트를 반환."""
        assigned_vehicle, new_path, cost = self.engine.assign_request(self.vehicles, request)
        
        if assigned_vehicle is None:
            print(f"[요청 {request.passenger_id}] 배차 실패")
            pickup_lon, pickup_lat = node_lonlat(self.graph, request.pickup_node)
            dropoff_lon, dropoff_lat = node_lonlat(self.graph, request.dropoff_node)
            print(f"  - 픽업 노드: {request.pickup_node} (경도: {pickup_lon}, 위도: {pickup_lat})")
            print(f"  - 드롭오프 노드: {request.dropoff_node} (경도: {dropoff_lon}, 위도: {dropoff_lat})")
            print()
            return None
        
        previous_path = assigned_vehicle.clone_path()
        start_node = assigned_vehicle.current_node
        
        prev_route_nodes = build_route_nodes(self.graph, start_node, previous_path)
        new_route_nodes = build_route_nodes(self.graph, start_node, new_path)
        
        event = AssignmentEvent(
            request=request,
            vehicle_id=assigned_vehicle.vehicle_id,
            vehicle_start_node=start_node,
            cost=cost,
            previous_path=previous_path,
            new_path=list(new_path),
            previous_route_nodes=prev_route_nodes,
            new_route_nodes=new_route_nodes,
            request_time=request_time,
        )
        
        assigned_vehicle.path = new_path
        
        # 터미널 출력 (좌표값 포함)
        pickup_lon, pickup_lat = node_lonlat(self.graph, request.pickup_node)
        dropoff_lon, dropoff_lat = node_lonlat(self.graph, request.dropoff_node)
        
        print(f"[요청 {request.passenger_id}] 차량 {assigned_vehicle.vehicle_id} 배정 (비용: {cost:,.2f}, 시간: {request_time:.1f}초)")
        print(f"  - 픽업 노드: {request.pickup_node} (경도: {pickup_lon}, 위도: {pickup_lat})")
        print(f"  - 드롭오프 노드: {request.dropoff_node} (경도: {dropoff_lon}, 위도: {dropoff_lat})")
        print(f"  - 기존 경로: {format_stop_sequence(previous_path)}")
        print(f"  - 신규 경로: {format_stop_sequence(new_path)}")
        print()
        
        return event

    def run_simulation(
        self,
        fixed_demands: List[FixedDemand],
        request_interval: float = REQUEST_INTERVAL_SECONDS,
    ) -> List[AssignmentEvent]:
        """
        고정된 디멘드 목록을 사용하여 시뮬레이션을 실행합니다.
        
        시뮬레이션 상에서는 request_interval 간격으로 요청이 들어오지만,
        실제 코드 실행은 빠르게 진행됩니다 (시간 지연 없음).
        
        Args:
            fixed_demands: 고정된 디멘드 목록
            request_interval: 시뮬레이션 상의 요청 간격 (초) - 실제 대기 시간은 없음
        
        Returns:
            배차 이벤트 목록
        """
        events: List[AssignmentEvent] = []
        current_time = 0.0
        
        print(f"[시뮬레이션 시작] 총 {len(fixed_demands)}개 디멘드, 시뮬레이션 요청 간격: {request_interval}초 (실제 실행은 빠르게 진행)\n")
        
        for idx, demand in enumerate(fixed_demands, 1):
            request = PassengerRequest(
                passenger_id=demand.passenger_id,
                pickup_node=demand.pickup_node,
                dropoff_node=demand.dropoff_node,
            )
            
            event = self.process_request(request, current_time)
            if event:
                events.append(event)
            
            # 시뮬레이션 시간만 증가 (실제 대기는 없음)
            current_time += request_interval
        
        print(f"[시뮬레이션 완료] 총 {len(events)}개 요청 배정 성공\n")
        return events


# --------------------------------------------------------------------------------------
# 메인 실행
# --------------------------------------------------------------------------------------


def main() -> None:
    """메인 실행 함수."""
    print("=" * 70)
    print("DRT 네트워크 배정 시뮬레이션 (고정된 디멘드)")
    print("=" * 70)
    print()
    
    # ================================================================================
    # 디멘드 개수 설정 확인
    # ================================================================================
    num_demands = NUM_DEMANDS
    print(f"[설정] 디멘드 개수: {num_demands}개")
    print(f"[설정] 요청 간격: {REQUEST_INTERVAL_SECONDS}초")
    print(f"[설정] 차량 수: {NUM_VEHICLES}대")
    print(f"[설정] 차량 용량: {VEHICLE_CAPACITY}명")
    print()
    # ================================================================================
    
    # 시뮬레이션 초기화
    simulation = FixedDemandSimulation(
        num_vehicles=NUM_VEHICLES,
        vehicle_capacity=VEHICLE_CAPACITY,
    )
    
    # 고정된 디멘드 생성
    fixed_demands = generate_fixed_demands(simulation.graph, num_demands, DEMAND_SEED)
    print(f"[디멘드 생성 완료] 총 {len(fixed_demands)}개 디멘드 생성\n")
    
    # 시뮬레이션 실행 (시뮬레이션 상으로는 5초 간격이지만 실제 실행은 빠르게 진행)
    events = simulation.run_simulation(
        fixed_demands,
        request_interval=REQUEST_INTERVAL_SECONDS,
    )
    
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

