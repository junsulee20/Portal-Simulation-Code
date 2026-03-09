"""
대기시간(wait time) 기반 3항 비용 함수를 적용한 DRT 배차 알고리즘 모듈.

원본: drt_network_assignment_optimized_sampling.py 에서 최소 수정.

핵심 변경:
    - ENABLE_WAITING_COST 플래그:
        OFF(False) → 기존 2항 비용 함수 100% 동일
        ON(True)   → 3항 비용 함수 사용
            final_cost = w1*cost_increase + w2*new_path_time + w3*total_wait
            total_wait = wait_assign + wait_pickup
                wait_assign : request_time ~ assigned_time (시뮬레이터에서 주입, 기본값 0)
                wait_pickup : assigned_time ~ pickup ETA (차량 현재 위치→픽업 노드 이동 시간)
    - DRTAssignmentEngine 생성 시 w1, w2, w3 및 wait_assign_seconds 파라미터 주입 가능
    - assign_request 반환값에 wait_pickup 추가 → (vehicle, new_path, cost, wait_pickup_sec)
"""

from __future__ import annotations

import math
import pickle
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx

# --------------------------------------------------------------------------------------
# 전역 설정 플래그
# --------------------------------------------------------------------------------------

# OFF → 기존 결과 100% 동일 / ON → 3항 비용 함수 사용
ENABLE_WAITING_COST: bool = True

# 기존 2항 비용 함수 가중치 (ENABLE_WAITING_COST=False 시 사용)
W_COST_INCREASE_LEGACY = 0.7
W_PATH_LENGTH_LEGACY   = 0.3

# 성능 최적화 설정 (원본과 동일)
MAX_PATH_LENGTH = 50
MAX_CANDIDATES_PER_VEHICLE = 20
EARLY_TERMINATION_THRESHOLD = 1.3

# --------------------------------------------------------------------------------------
# 데이터 모델 (원본과 동일)
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Stop:
    """차량 경로 상의 단일 정차 지점."""

    node_id: int
    stop_type: str   # "pickup" 또는 "dropoff"
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
# 네트워크 기반 이동 시간 계산기 (원본과 동일)
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


@dataclass(frozen=True)
class Candidate:
    """삽입 후보를 나타내는 데이터 클래스."""

    pickup_index: int
    dropoff_index: int


class DRTAssignmentEngine:
    """
    DRT 1:다수 배정 알고리즘 (3항 비용 함수 옵션 추가).

    Parameters
    ----------
    graph : nx.Graph
    max_path_length : int
    w1, w2, w3 : float
        3항 비용 함수 가중치 (w1+w2+w3 = 1).
        ENABLE_WAITING_COST=False 이면 무시됩니다.
    """

    def __init__(
        self,
        graph: nx.Graph,
        max_path_length: int = MAX_PATH_LENGTH,
        w1: float = 0.7,
        w2: float = 0.3,
        w3: float = 0.0,
    ) -> None:
        self.graph = graph
        self.travel_time_cache = NetworkTravelTimeCache(graph)
        self.max_path_length = max_path_length
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3

    # 공개 API -------------------------------------------------------------------------
    def assign_request(
        self,
        vehicles: Sequence[VehicleState],
        request: PassengerRequest,
        wait_assign_seconds: float = 0.0,
    ) -> Tuple[Optional[VehicleState], List[Stop], float, float]:
        """
        신규 승객 요청을 받아 가장 적합한 차량과 업데이트된 경로, 최종 비용, 픽업 대기 시간을 반환.

        Parameters
        ----------
        vehicles : Sequence[VehicleState]
        request  : PassengerRequest
        wait_assign_seconds : float
            wait_assign = 요청 시각 ~ 배정 완료 시각 (초). 시뮬레이터에서 주입.

        Returns
        -------
        (선정된 차량, 신규 경로, 최종 비용, wait_pickup_seconds)
        배차 실패 시 (None, [], math.inf, math.inf)
        """
        best_vehicle: Optional[VehicleState] = None
        best_new_path: List[Stop] = []
        min_final_cost: float = math.inf
        best_wait_pickup: float = math.inf

        for vehicle in vehicles:
            if vehicle.onboard_passengers >= vehicle.capacity:
                continue
            if len(vehicle.path) > self.max_path_length:
                continue

            original_path_time = self._calculate_path_time(vehicle.current_node, vehicle.path)
            if math.isinf(original_path_time):
                continue

            candidate_path, candidate_cost, candidate_wait_pickup = \
                self._find_best_insertion_with_sampling(
                    vehicle, request, original_path_time, wait_assign_seconds
                )
            if candidate_path is None:
                continue

            if candidate_cost < min_final_cost:
                best_vehicle = vehicle
                best_new_path = candidate_path
                min_final_cost = candidate_cost
                best_wait_pickup = candidate_wait_pickup

        if best_vehicle is None:
            return None, [], math.inf, math.inf

        return best_vehicle, best_new_path, min_final_cost, best_wait_pickup

    # 내부 메서드 ----------------------------------------------------------------------
    def _find_best_insertion_with_sampling(
        self,
        vehicle: VehicleState,
        request: PassengerRequest,
        original_path_time: float,
        wait_assign_seconds: float,
    ) -> Tuple[Optional[List[Stop]], float, float]:
        """
        주어진 차량 경로에서 픽업·드롭오프를 삽입할 최적 위치와 비용을 탐색.

        Returns
        -------
        (best_path, best_cost, best_wait_pickup)
        """
        path_len = len(vehicle.path)

        if path_len + 2 > self.max_path_length:
            return None, math.inf, math.inf

        original_path_nodes = self._get_path_nodes(vehicle.current_node, vehicle.path)

        # 모든 후보 조합 생성
        all_candidates: List[Candidate] = []
        for pickup_index in range(path_len + 1):
            for dropoff_index in range(pickup_index + 1, path_len + 2):
                all_candidates.append(Candidate(pickup_index=pickup_index, dropoff_index=dropoff_index))

        # 균등 샘플링
        if len(all_candidates) > MAX_CANDIDATES_PER_VEHICLE:
            sampled_candidates = random.sample(all_candidates, MAX_CANDIDATES_PER_VEHICLE)
        else:
            sampled_candidates = all_candidates

        best_path: Optional[List[Stop]] = None
        best_cost: float = math.inf
        best_wait_pickup: float = math.inf

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

            if len(path_candidate) > self.max_path_length:
                continue

            if not self._is_capacity_valid(path_candidate, vehicle.capacity, vehicle.onboard_passengers):
                continue

            # 경로 시간 계산 + 픽업까지의 ETA
            if path_len <= 5:
                new_path_time, wait_pickup = self._calculate_path_time_with_pickup_eta(
                    vehicle.current_node, path_candidate, request.pickup_node
                )
            else:
                new_path_time, wait_pickup = self._calculate_path_time_incremental_with_eta(
                    vehicle.current_node,
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

            # -----------------------------------------------------------------------
            # 비용 함수 선택
            # -----------------------------------------------------------------------
            if ENABLE_WAITING_COST:
                total_wait = wait_assign_seconds + wait_pickup  # 단위: 초
                final_cost = self.w1 * cost_increase + self.w2 * new_path_time + self.w3 * total_wait
            else:
                # 기존 2항 비용 함수 (100% 동일)
                final_cost = W_COST_INCREASE_LEGACY * cost_increase + W_PATH_LENGTH_LEGACY * new_path_time
            # -----------------------------------------------------------------------

            if best_cost != math.inf and final_cost > best_cost * EARLY_TERMINATION_THRESHOLD:
                continue

            if final_cost < best_cost:
                best_cost = final_cost
                best_path = path_candidate
                best_wait_pickup = wait_pickup

        return best_path, best_cost, best_wait_pickup

    def _calculate_path_time_with_pickup_eta(
        self,
        start_node: int,
        path: List[Stop],
        pickup_node: int,
    ) -> Tuple[float, float]:
        """
        전체 경로 이동 시간과 픽업 노드까지 도달하는 시간을 함께 계산.

        Returns
        -------
        (total_path_time_seconds, wait_pickup_seconds)
        """
        total_seconds = 0.0
        current_node = start_node
        wait_pickup = math.inf

        for stop in path:
            travel_seconds = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel_seconds):
                return math.inf, math.inf
            total_seconds += travel_seconds
            current_node = stop.node_id

            # 픽업 노드 최초 도달 시각 기록
            if wait_pickup == math.inf and stop.node_id == pickup_node and stop.stop_type == "pickup":
                wait_pickup = total_seconds

        if wait_pickup == math.inf:
            wait_pickup = 0.0  # 픽업 스톱을 경로에서 찾지 못하는 경우 방어
        return total_seconds, wait_pickup

    def _calculate_path_time_incremental_with_eta(
        self,
        start_node: int,
        original_path: List[Stop],
        original_path_nodes: List[int],
        original_path_time: float,
        pickup_index: int,
        pickup_node: int,
        dropoff_index: int,
        dropoff_node: int,
    ) -> Tuple[float, float]:
        """
        증분 계산 버전: 전체 경로 이동 시간 + 픽업 ETA.

        Returns
        -------
        (total_path_time_seconds, wait_pickup_seconds)
        """
        total_time = 0.0
        current_node = start_node

        # pickup_index 이전까지 (원본 그대로)
        for i in range(pickup_index):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf, math.inf
            total_time += travel
            current_node = stop.node_id

        # 픽업 노드
        pickup_travel = self.travel_time_cache.travel_seconds(current_node, pickup_node)
        if math.isinf(pickup_travel):
            return math.inf, math.inf
        total_time += pickup_travel
        wait_pickup = total_time           # <- 픽업까지 소요 시간
        current_node = pickup_node

        # 픽업~드롭오프 사이 원본 경로
        for i in range(pickup_index, dropoff_index - 1):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf, math.inf
            total_time += travel
            current_node = stop.node_id

        # 드롭오프 노드
        dropoff_travel = self.travel_time_cache.travel_seconds(current_node, dropoff_node)
        if math.isinf(dropoff_travel):
            return math.inf, math.inf
        total_time += dropoff_travel
        current_node = dropoff_node

        # 드롭오프 이후 원본 경로
        for i in range(dropoff_index - 1, len(original_path)):
            stop = original_path[i]
            travel = self.travel_time_cache.travel_seconds(current_node, stop.node_id)
            if math.isinf(travel):
                return math.inf, math.inf
            total_time += travel
            current_node = stop.node_id

        return total_time, wait_pickup

    def _get_path_nodes(self, start_node: int, path: List[Stop]) -> List[int]:
        """경로의 노드 시퀀스를 반환 (증분 계산용)."""
        nodes = [start_node]
        for stop in path:
            nodes.append(stop.node_id)
        return nodes

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
# 유틸리티 함수 (원본과 동일)
# --------------------------------------------------------------------------------------


def load_network_graph(path: Optional[Path] = None) -> nx.Graph:
    """`simulation/network/main_network_graph.pkl` 파일을 로드해 그래프 객체 반환."""
    if path is None:
        # 이 파일(__file__)의 위치에서 부모 디렉토리(Portal-Simulation-Code)로 올라가서 탐색
        # DRT/ 폴더 기준 → 상위 폴더 / simulation / network / ...
        _here = Path(__file__).resolve().parent
        _candidate = _here.parent / "simulation" / "network" / "main_network_graph.pkl"
        if _candidate.exists():
            path = _candidate
        else:
            # 기존 방식(CWD 기준) 폴백
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
