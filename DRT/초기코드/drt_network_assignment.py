"""
실제 `main_network_graph.pkl` 네트워크 상에서 동작하는 1:다수 DRT 배차 알고리즘 예제 모듈.

핵심 아이디어:
    * `DRT_V3_objective_function.py`의 휴리스틱을 기반으로 함.
    * 실제 네트워크 그래프(가중치가 포함된 NetworkX 그래프)를 이용해 이동 시간(또는 거리)을 계산.
    * 승객 한 명의 픽업·드롭오프를 기존 차량 경로의 모든 가능한 위치에 삽입하며
      비용 함수(`W_COST_INCREASE`, `W_PATH_LENGTH`)를 최소화하는 차량을 선택.

사용 방법:
    1. `load_network_graph()`를 이용해 `simulation/network/main_network_graph.pkl`을 로드합니다.
    2. `DRTAssignmentEngine` 인스턴스를 생성하고, 차량 목록(`VehicleState`)과 신규 승객 요청(`PassengerRequest`)을 전달해
       `assign_request()`를 호출합니다.
    3. 반환값으로 최적 차량과 업데이트된 경로를 받아서 시스템 상태를 갱신합니다.

본 모듈은 예제용이며, 실제 운영 환경에 맞게 입출력 구조나 상태 관리 코드를 추가하면 됩니다.
"""

from __future__ import annotations

import math
import pickle
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx

# 비용 함수 가중치 (기존 DRT 로직과 동일)
W_COST_INCREASE = 0.7
W_PATH_LENGTH = 0.3

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


@dataclass
class VehicleState:
    """DRT 배차 알고리즘이 사용하는 차량 상태."""

    vehicle_id: int
    current_node: int
    capacity: int
    onboard_passengers: int = 0
    path: List[Stop] = field(default_factory=list)

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
# DRT 1:다수 배정 엔진
# --------------------------------------------------------------------------------------


class DRTAssignmentEngine:
    """
    DRT 1:다수 배정 알고리즘을 실제 네트워크에 적용한 엔진.

    차량별 경로를 시뮬레이션하지 않고도 신규 요청을 어느 차량에 배치할지 결정할 수 있습니다.
    """

    def __init__(self, graph: nx.Graph) -> None:
        self.graph = graph
        self.travel_time_cache = NetworkTravelTimeCache(graph)

    # 공개 API ------------------------------------------------------------------------
    def assign_request(
        self,
        vehicles: Sequence[VehicleState],
        request: PassengerRequest,
    ) -> Tuple[Optional[VehicleState], List[Stop], float]:
        """
        신규 승객 요청을 받아 가장 적합한 차량과 업데이트된 경로, 계산된 최종 비용을 반환.

        반환값:
            (선정된 차량 객체, 신규 경로 리스트, 최종 비용)
            배차 실패 시 (None, [], math.inf)
        """
        best_vehicle: Optional[VehicleState] = None
        best_new_path: List[Stop] = []
        min_final_cost: float = math.inf

        for vehicle in vehicles:
            if vehicle.onboard_passengers >= vehicle.capacity:
                continue

            original_path_time = self._calculate_path_time(vehicle.current_node, vehicle.path)
            if math.isinf(original_path_time):
                # 현재 경로조차 유효하게 계산되지 않으면 해당 차량은 배제
                continue

            candidate_path, candidate_cost = self._find_best_insertion(vehicle, request, original_path_time)
            if candidate_path is None:
                continue

            if candidate_cost < min_final_cost:
                best_vehicle = vehicle
                best_new_path = candidate_path
                min_final_cost = candidate_cost

        if best_vehicle is None:
            return None, [], math.inf

        return best_vehicle, best_new_path, min_final_cost

    # 내부 메서드 ---------------------------------------------------------------------
    def _find_best_insertion(
        self,
        vehicle: VehicleState,
        request: PassengerRequest,
        original_path_time: float,
    ) -> Tuple[Optional[List[Stop]], float]:
        """주어진 차량 경로에서 픽업·드롭오프를 삽입할 최적 위치와 비용을 탐색."""
        best_path: Optional[List[Stop]] = None
        best_cost: float = math.inf

        path_len = len(vehicle.path)

        for pickup_index in range(path_len + 1):
            path_with_pickup = vehicle.clone_path()
            path_with_pickup.insert(
                pickup_index,
                Stop(node_id=request.pickup_node, stop_type="pickup", passenger_id=request.passenger_id),
            )

            for dropoff_index in range(pickup_index + 1, len(path_with_pickup) + 1):
                path_candidate = list(path_with_pickup)
                path_candidate.insert(
                    dropoff_index,
                    Stop(node_id=request.dropoff_node, stop_type="dropoff", passenger_id=request.passenger_id),
                )

                if not self._is_capacity_valid(path_candidate, vehicle.capacity, vehicle.onboard_passengers):
                    continue

                new_path_time = self._calculate_path_time(vehicle.current_node, path_candidate)
                if math.isinf(new_path_time):
                    continue

                cost_increase = new_path_time - original_path_time
                final_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_path_time)

                if final_cost < best_cost:
                    best_cost = final_cost
                    best_path = path_candidate

        return best_path, best_cost

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

