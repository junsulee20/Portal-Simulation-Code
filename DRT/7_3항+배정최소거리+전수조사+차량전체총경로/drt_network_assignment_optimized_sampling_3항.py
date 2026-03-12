"""
실제 `main_network_graph.pkl` 네트워크 상에서 동작하는 1:다수 DRT 배차 알고리즘 예제 모듈 (전수 조사 버전).

성능 최적화:
    - 경로 길이 제한: MAX_PATH_LENGTH=50로 제한하여 계산량 감소
    - 전수 조사: 샘플링 없이 모든 삽입 후보를 평가하여 수학적으로 동일한 최적값 보장
    - 조기 종료: 현재 최적해의 1.3배 이상인 후보는 즉시 건너뛰기
    - 증분 계산: 경로가 길 때 전체 경로를 다시 계산하지 않고 증가분만 계산
    - 적응형 계산: 경로가 짧으면 전체 계산, 길면 증분 계산 사용
"""

from __future__ import annotations

import math
import pickle
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx

# 비용 함수 가중치
W_COST_INCREASE = 0.5  # w1: 경로 비용 증가량
W_PATH_LENGTH = 0   # w2: 신규 경로 전체 시간
W_WAIT_TIME = 0.5      # w3: 대기시간 (wait_assign + wait_pickup)

# 성능 최적화 및 배정 조건 설정
MAX_PATH_LENGTH = 50  # 경로 길이 제한 (stop 개수) - 100개 요청 처리 가능하도록 증가
EARLY_TERMINATION_THRESHOLD = 1.3  # 조기 종료 임계값: 현재 최적해의 1.3배 이상이면 건너뛰기 (더 공격적)

MAX_DISPATCH_ETA_SECONDS = 100000  # 차량 배정 시 허용되는 최대 픽업 ETA (초 단위). 이 시간(거리) 이내에 차량이 있을 때만 배정됨.

# --------------------------------------------------------------------------------------
# 데이터 모델
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Stop:
    """차량 경로 상의 단일 정차 지점."""

    node_id: int
    stop_type: str  # "pickup" 또는 "dropoff"
    passenger_id: str

    def __post_init__(self) -> None:
        if self.stop_type not in ("pickup", "dropoff"):
            raise ValueError(f"stop_type은 'pickup' 또는 'dropoff'여야 합니다. 입력값: {self.stop_type}")


@dataclass
class PassengerRequest:
    """신규 승객 요청."""

    passenger_id: str
    pickup_node: int
    dropoff_node: int
    request_time: float = 0.0   # 요청 시각 (초 단위)
    assigned_time: float = 0.0  # 배정 시각 (초 단위); assign_request 호출 시점에 설정


@dataclass
class VehicleState:
    """DRT 배차 알고리즘이 사용하는 차량 상태."""

    vehicle_id: int
    current_node: int
    capacity: int
    onboard_passengers: int = 0
    path: List[Stop] = field(default_factory=list)
    depot_node: Optional[int] = None  # 차고지(초기 출발지) 노드 추가

    def __post_init__(self) -> None:
        if self.depot_node is None:
            self.depot_node = self.current_node

    def clone_path(self) -> List[Stop]:
        """경로 리스트를 깊은 복사(Stop은 불변 객체로 취급)하여 반환."""
        return list(self.path)


# --------------------------------------------------------------------------------------
# 네트워크 기반 이동 시간 계산기
# --------------------------------------------------------------------------------------


class NetworkTravelTimeCache:
    """
    NetworkX 그래프를 이용해 두 노드 간 최단 경로 시간을 계산하고 캐싱.

    `main_network_graph.pkl`의 `weight`는 분 단위로 추정되므로,
    내부적으로는 초(second) 단위로 변환하여 반환합니다.
    """

    def __init__(self, graph: nx.Graph) -> None:
        self.graph = graph
        self._cache: Dict[Tuple[int, int], float] = {}

    def travel_seconds(self, source: int, target: int) -> float:
        """source→target 최단 경로 시간을 초 단위로 반환. 경로가 없으면 inf."""
        if source == target:
            return 0.0

        key = (source, target)
        if key not in self._cache:
            try:
                minutes = nx.shortest_path_length(self.graph, source=source, target=target, weight="weight")
                self._cache[key] = float(minutes) * 60.0
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                self._cache[key] = math.inf
        return self._cache[key]


# --------------------------------------------------------------------------------------
# DRT 1:다수 배정 엔진 (차량별 균등 샘플링 버전)
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Candidate:
    """삽입 후보를 나타내는 데이터 클래스."""

    pickup_index: int
    dropoff_index: int


class DRTAssignmentEngine:
    """
    DRT 1:다수 배정 알고리즘을 실제 네트워크에 적용한 엔진 (전수 조사 버전).

    차량별 경로를 시뮬레이션하지 않고도 신규 요청을 어느 차량에 배치할지 결정할 수 있습니다.
    
    성능 최적화:
        - 경로 길이 제한: MAX_PATH_LENGTH를 초과하는 경로는 배제
        - 전수 조사: 모든 삽입 후보를 평가하여 수학적 최적해 보장
        - 조기 종료: 최적해보다 나쁜 후보는 즉시 건너뛰기
    """

    def __init__(self, graph: nx.Graph, max_path_length: int = MAX_PATH_LENGTH, max_dispatch_eta: float = MAX_DISPATCH_ETA_SECONDS) -> None:
        self.graph = graph
        self.travel_time_cache = NetworkTravelTimeCache(graph)
        self.max_path_length = max_path_length
        self.max_dispatch_eta = max_dispatch_eta

    # 공개 API ------------------------------------------------------------------------
    def assign_request(
        self,
        vehicles: Sequence[VehicleState],
        request: PassengerRequest,
        current_time: float = 0.0,
    ) -> Tuple[Optional[VehicleState], List[Stop], float]:
        """
        신규 승객 요청을 받아 가장 적합한 차량과 업데이트된 경로, 계산된 최종 비용을 반환.

        전수 조사: 각 차량의 모든 삽입 후보를 평가하여 수학적으로 동일한 최적값을 반환합니다.

        인수:
            vehicles     : 평가할 차량 목록
            request      : 신규 승객 요청 (request_time 포함)
            current_time : 현재 시각(초). 배정 시각(assigned_time) 계산에 사용됨.

        반환값:
            (선정된 차량 객체, 신규 경로 리스트, 최종 비용)
            배차 실패 시 (None, [], math.inf)
        """
        best_vehicle: Optional[VehicleState] = None
        best_new_path: List[Stop] = []
        min_final_cost: float = math.inf

        # 배정 시각: assign_request가 호출된 시점 = current_time
        assigned_time = current_time
        # wait_assign: 요청(request_time) → 배정(assigned_time)
        wait_assign = max(0.0, assigned_time - request.request_time)

        # 전체 차량(5대)의 기존 누적 경로 시간 합계 계산
        total_original_system_path_time = 0.0
        for v in vehicles:
            v_time = self._calculate_path_time(v.depot_node, v.path)
            if not math.isinf(v_time):
                total_original_system_path_time += v_time

        # 각 차량별로 후보를 수집하고 샘플링
        for vehicle in vehicles:
            if vehicle.onboard_passengers >= vehicle.capacity:
                continue

            # 성능 최적화: 경로 길이 제한
            if len(vehicle.path) > self.max_path_length:
                continue

            # 차고지(depot)에서부터의 전체 경로 총 누적 운행 시간 계산
            original_path_time = self._calculate_path_time(vehicle.depot_node, vehicle.path)
            if math.isinf(original_path_time):
                # 현재 경로조차 유효하게 계산되지 않으면 해당 차량은 배제
                continue

            # 차량별 균등 샘플링: 모든 후보를 생성하고 샘플링
            candidate_path, candidate_cost, pickup_eta = self._find_best_insertion_full(
                vehicle, request, original_path_time, total_original_system_path_time
            )
            if candidate_path is None:
                continue

            # 최소 조건: 픽업 ETA가 허용된 최대 픽업 시간(최대 배정 거리) 이내일 때만 배정 허용
            if pickup_eta > self.max_dispatch_eta:
                continue

            # wait_pickup: 배정(assigned_time) → 탑승/픽업 ETA
            # pickup_eta는 차량이 현재 위치에서 픽업 지점까지 도달하는 데 걸리는 시간(초)
            wait_pickup = max(0.0, pickup_eta)
            total_wait = wait_assign + wait_pickup

            # 최종 목적함수: w1*cost_increase + w2*new_path_time + w3*total_wait
            # candidate_cost 는 (w1*cost_increase + w2*new_path_time) 항만 포함하므로
            # 여기서 w3 항을 추가
            final_cost = candidate_cost + W_WAIT_TIME * total_wait

            if final_cost < min_final_cost:
                best_vehicle = vehicle
                best_new_path = candidate_path
                min_final_cost = final_cost

        if best_vehicle is None:
            return None, [], math.inf

        return best_vehicle, best_new_path, min_final_cost

    # 내부 메서드 ---------------------------------------------------------------------
    def _find_best_insertion_full(
        self,
        vehicle: VehicleState,
        request: PassengerRequest,
        original_path_time: float,
        total_original_system_path_time: float,
    ) -> Tuple[Optional[List[Stop]], float, float]:
        """
        주어진 차량 경로에서 픽업·드롭오프를 삽입할 최적 위치와 비용을 탐색 (전수 조사).

        반환값:
            (최적 경로 | None,
             w1*cost_increase + w2*new_path_time 항의 부분 비용,
             픽업 ETA(초) — 차량 현재 위치 → 픽업 노드 도달까지 걸리는 시간)

        성능 최적화:
            - 경로 길이 제한: MAX_PATH_LENGTH를 초과하는 후보는 배제
            - 전수 조사: 모든 후보를 평가하여 수학적 최적해 보장
            - 증분 계산: 전체 경로를 다시 계산하지 않고 증가분만 계산
            - 조기 종료: 최적해보다 나쁜 후보는 즉시 건너뛰기
        """
        path_len = len(vehicle.path)
        
        # 성능 최적화: 경로 길이 제한
        if path_len + 2 > self.max_path_length:
            # 새 경로가 제한을 초과하면 배제
            return None, math.inf, math.inf

        # 성능 최적화: 원본 경로의 중간 노드 위치를 미리 계산 (증분 계산용)
        # 누적 운행 시간 계산을 위해 차고지(depot_node)를 시작점으로 사용
        original_path_nodes = self._get_path_nodes(vehicle.depot_node, vehicle.path)
        
        # 모든 가능한 후보 조합 생성
        all_candidates: List[Candidate] = []
        for pickup_index in range(path_len + 1):
            for dropoff_index in range(pickup_index + 1, path_len + 2):  # path_with_pickup 길이는 path_len + 1
                all_candidates.append(Candidate(pickup_index=pickup_index, dropoff_index=dropoff_index))
        
        # 전수 조사: 샘플링 없이 모든 후보 평가 (MAX_PATH_LENGTH=50 제한으로 인해 계산량 관리됨)
        sampled_candidates = all_candidates
        
        # 모든 후보를 평가
        best_path: Optional[List[Stop]] = None
        best_partial_cost: float = math.inf   # w1*cost_increase + w2*new_path_time 항
        best_pickup_eta: float = math.inf
        
        for candidate in sampled_candidates:
            path_with_pickup = vehicle.clone_path()
            path_with_pickup.insert(
                candidate.pickup_index,
                Stop(node_id=request.pickup_node, stop_type="pickup", passenger_id=request.passenger_id),
            )
            
            path_candidate = list(path_with_pickup)
            path_candidate.insert(
                candidate.dropoff_index,
                Stop(node_id=request.dropoff_node, stop_type="dropoff", passenger_id=request.passenger_id),
            )

            # 성능 최적화: 경로 길이 제한
            if len(path_candidate) > self.max_path_length:
                continue

            if not self._is_capacity_valid(path_candidate, vehicle.capacity, vehicle.onboard_passengers):
                continue

            # 성능 최적화: 증분 계산 사용 (경로가 짧으면 전체 계산이 더 빠를 수 있음)
            if path_len <= 5:
                # 차고지(depot_node)에서부터 전체 누적 운행 시간 다시 계산
                new_path_time = self._calculate_path_time(vehicle.depot_node, path_candidate)
            else:
                # 경로가 길면 증분 계산 사용 (차고지 출발 기준)
                new_path_time = self._calculate_path_time_incremental(
                    vehicle.depot_node,
                    vehicle.path,
                    original_path_nodes,
                    original_path_time,
                    candidate.pickup_index,
                    request.pickup_node,
                    candidate.dropoff_index,
                    request.dropoff_node,
                )
            
            if math.isinf(new_path_time):
                continue

            cost_increase = new_path_time - original_path_time
            
            # 차량 전체(5대) 총 경로 소요 시간 계산
            system_new_path_time = total_original_system_path_time - original_path_time + new_path_time

            # w1, w2 항만 계산 (w3 항은 assign_request에서 추가)
            partial_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * system_new_path_time)

            # 픽업 ETA: 차고지(depot) → 픽업까지 경유하는 스톱들을 따라 이동한 시간
            pickup_eta = self._calculate_pickup_eta(
                vehicle.depot_node, vehicle.path, candidate.pickup_index, request.pickup_node
            )

            # 성능 최적화: 조기 종료 - 현재 최적해보다 훨씬 나쁘면 건너뛰기
            if best_partial_cost != math.inf and partial_cost > best_partial_cost * EARLY_TERMINATION_THRESHOLD:
                continue

            if partial_cost < best_partial_cost:
                best_partial_cost = partial_cost
                best_path = path_candidate
                best_pickup_eta = pickup_eta

        return best_path, best_partial_cost, best_pickup_eta

    def _calculate_pickup_eta(
        self,
        start_node: int,
        original_path: List[Stop],
        pickup_index: int,
        pickup_node: int,
    ) -> float:
        """
        픽업 노드까지 도달하는 데 걸리는 시간(초).
        """
        total_time = 0.0
        current_node = start_node

        # 픽업 삽입 위치 이전까지 기존 경로를 따라 이동
        for i in range(pickup_index):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf
            total_time += travel
            current_node = stop.node_id

        # 픽업 노드까지 이동
        pickup_travel = self.travel_time_cache.travel_seconds(current_node, pickup_node)
        if math.isinf(pickup_travel):
            return math.inf
        total_time += pickup_travel

        return total_time
    
    def _get_path_nodes(self, start_node: int, path: List[Stop]) -> List[int]:
        """경로의 노드 시퀀스를 반환 (증분 계산용)."""
        nodes = [start_node]
        for stop in path:
            nodes.append(stop.node_id)
        return nodes
    
    def _calculate_path_time_incremental(
        self,
        start_node: int,
        original_path: List[Stop],
        original_path_nodes: List[int],
        original_path_time: float,
        pickup_index: int,
        pickup_node: int,
        dropoff_index: int,
        dropoff_node: int,
    ) -> float:
        """
        증분 계산: 전체 경로를 다시 계산하지 않고 삽입 지점 주변만 계산.
        
        원본 경로의 노드 시퀀스를 활용하여 삽입 지점 이전/이후를 효율적으로 계산.
        """
        total_time = 0.0
        current_node = start_node
        
        # pickup_index 이전까지의 경로 (원본 경로 그대로)
        for i in range(pickup_index):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf
            total_time += travel
            current_node = stop.node_id
        
        # 픽업 노드까지의 이동
        pickup_travel = self.travel_time_cache.travel_seconds(current_node, pickup_node)
        if math.isinf(pickup_travel):
            return math.inf
        total_time += pickup_travel
        current_node = pickup_node
        
        # 드롭오프 이전까지의 경로 (픽업과 드롭오프 사이의 원본 경로)
        # dropoff_index는 path_with_pickup 기준이므로, 원본 경로에서는 dropoff_index - 1까지
        for i in range(pickup_index, dropoff_index - 1):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf
            total_time += travel
            current_node = stop.node_id
        
        # 드롭오프 노드까지의 이동
        dropoff_travel = self.travel_time_cache.travel_seconds(current_node, dropoff_node)
        if math.isinf(dropoff_travel):
            return math.inf
        total_time += dropoff_travel
        current_node = dropoff_node
        
        # 드롭오프 이후의 경로 (원본 경로의 나머지)
        for i in range(dropoff_index - 1, len(original_path)):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf
            total_time += travel
            current_node = stop.node_id
        
        return total_time

    def _calculate_path_time(self, start_node: int, path: Iterable[Stop]) -> float:
        """주어진 경로(Stop 목록)를 따라 이동하는 데 필요한 총 시간을 초 단위로 계산."""
        total_seconds = 0.0
        current_node = start_node

        for stop in path:
            travel_seconds = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel_seconds):
                return math.inf
            total_seconds += travel_seconds
            current_node = stop.node_id

        return total_seconds

    @staticmethod
    def _is_capacity_valid(path: Sequence[Stop], capacity: int, current_onboard: int) -> bool:
        """경로 수행 중 차량 용량을 초과하지 않는지 확인."""
        onboard = current_onboard
        for stop in path:
            if stop.stop_type == "pickup":
                onboard += 1
            elif stop.stop_type == "dropoff":
                onboard = max(0, onboard - 1)
            if onboard > capacity:
                return False
        return True


# --------------------------------------------------------------------------------------
# 유틸리티 함수
# --------------------------------------------------------------------------------------


def load_network_graph(path: Optional[Path] = None) -> nx.Graph:
    """`simulation/network/main_network_graph.pkl` 파일을 로드해 그래프 객체 반환."""
    if path is None:
        path = Path("simulation/network/main_network_graph.pkl")

    with path.open("rb") as f:
        graph = pickle.load(f)

    if not isinstance(graph, nx.Graph):
        raise TypeError(f"피클 로드 결과가 NetworkX 그래프가 아닙니다: {type(graph)}")

    return graph


def select_random_node(graph: nx.Graph) -> int:
    """그래프에서 임의의 노드 하나를 반환 (데모용)."""
    nodes = list(graph.nodes)
    return random.choice(nodes)


# --------------------------------------------------------------------------------------
# 실행 예제
# --------------------------------------------------------------------------------------


def run_demo(num_requests: int = 8) -> None:
    """
    네트워크를 불러와 무작위 승객 요청을 생성하고 배차 결과를 출력하는 간단한 예제.

    실제 시스템에 맞게 승객·차량 데이터를 주입하는 부분만 교체하면 배차 로직을 재사용할 수 있습니다.
    """
    graph = load_network_graph()
    engine = DRTAssignmentEngine(graph)

    # 데모용 차량 두 대 생성 (그래프 상 임의 노드 기준)
    vehicles = [
        VehicleState(vehicle_id=1, current_node=select_random_node(graph), capacity=4),
        VehicleState(vehicle_id=2, current_node=select_random_node(graph), capacity=4),
    ]

    for idx in range(1, num_requests + 1):
        pickup = select_random_node(graph)
        dropoff = select_random_node(graph)
        while dropoff == pickup:
            dropoff = select_random_node(graph)

        request = PassengerRequest(
            passenger_id=f"demo_{idx:03d}",
            pickup_node=pickup,
            dropoff_node=dropoff,
        )

        assigned_vehicle, new_path, cost = engine.assign_request(vehicles, request)

        if assigned_vehicle is None:
            print(f"[요청 {request.passenger_id}] 배차 실패")
            continue

        print(
            f"[요청 {request.passenger_id}] 차량 {assigned_vehicle.vehicle_id} 배정 "
            f"(최종 비용: {cost:,.2f}, 경로 길이: {len(new_path)} 스톱)"
        )

        # 실제 시스템에서는 여기서 DB 업데이트 등 후속 처리를 수행
        assigned_vehicle.path = new_path


if __name__ == "__main__":
    run_demo()