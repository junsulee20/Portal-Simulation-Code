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
W_PATH_LENGTH = 0.3    # w2: 신규 경로 전체 시간
W_WAIT_TIME = 0.2      # w3: 대기시간 (wait_assign + wait_pickup)

# 성능 최적화 및 배정 조건 설정
MAX_PATH_LENGTH = 50  # 경로 길이 제한 (stop 개수) - 100개 요청 처리 가능하도록 증가
EARLY_TERMINATION_THRESHOLD = 1.3  # 조기 종료 임계값: 현재 최적해의 1.3배 이상이면 건너뛰기 (더 공격적)

MAX_DISPATCH_ETA_SECONDS = 300  # 차량 배정 시 허용되는 최대 픽업 ETA (초 단위). 이 시간(거리) 이내에 차량이 있을 때만 배정됨.

# --------------------------------------------------------------------------------------
# 데이터 모델
# --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Stop:
    """차량 경로 상의 단일 정차 지점."""

    node_id: int
    stop_type: str  # "pickup" 또는 "dropoff"
    passenger_id: str
    is_street_hail: bool = False # 길거리 승객 여부

    def __post_init__(self) -> None:
        if self.stop_type not in ("pickup", "dropoff"):
            raise ValueError(f"stop_type은 'pickup' 또는 'dropoff'여야 합니다. 입력값: {self.stop_type}")


@dataclass
class PassengerRequest:
    """신규 승객 일반/길거리 요청."""

    passenger_id: str
    pickup_node: int
    dropoff_node: int
    request_time: float = 0.0   # 요청 시각 (초 단위)
    assigned_time: float = 0.0  # 배정 시각 (초 단위); assign_request 호출 시점에 설정
    is_street_hail: bool = False # 추가: 길거리에서 대기하는 승객 여부
    street_fail_reason: str = "경로상_차량_부재" # 추가: 길거리 탑승 실패 사유


@dataclass
class VehicleState:
    """DRT 배차 알고리즘이 사용하는 차량 상태."""

    vehicle_id: int
    current_node: int
    capacity: int
    onboard_passengers: int = 0
    path: List[Stop] = field(default_factory=list)
    depot_node: Optional[int] = None  # 차고지(초기 출발지) 노드 추가
    schedule_start_time: float = 0.0  # [동적이동] 현재 path가 할당된 시각 (초). 차량 위치 계산의 기준.
    reached_history: List[Tuple[Stop, float]] = field(default_factory=list) # 추가: 실제로 도달한 [정차지점, 도달시각] 기록

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
    
    주요 로직:
        - 일반 배정: 차량의 현재 경로 내 모든 삽입 위치를 평가하여 목적함수 최소화
        - 길거리 배정: 현재 주행 중인 길목에서 승객을 픽업할 수 있는 차량을 우선 검색
        - 유휴 이동 처리: "IDLE_MOVE" 상태의 차량은 경로가 없는 유휴 상태로 간주하여 즉시 배정

    성능 최적화:
        - 경로 길이 제한: MAX_PATH_LENGTH를 초과하는 경로는 배제
        - 전수 조사: 모든 삽입 후보를 평가하여 수학적 최적해 보장
        - 조기 종료: 최적해보다 나쁜 후보는 즉시 건너뛰기 (EARLY_TERMINATION_THRESHOLD 활용)
    """

    def __init__(self, graph: nx.Graph, max_path_length: int = MAX_PATH_LENGTH, max_dispatch_eta: float = MAX_DISPATCH_ETA_SECONDS, street_hail_travel_time_increase_limit: float = 300.0) -> None:
        self.graph = graph
        self.travel_time_cache = NetworkTravelTimeCache(graph)
        self.max_path_length = max_path_length
        self.max_dispatch_eta = max_dispatch_eta
        self.street_hail_travel_time_increase_limit = street_hail_travel_time_increase_limit

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

        # 각 차량별로 후보를 수집하고 샘플링
        for vehicle in vehicles:
            if vehicle.onboard_passengers >= vehicle.capacity:
                continue

            # 성능 최적화: 경로 길이 제한
            if len(vehicle.path) > self.max_path_length:
                continue

            # 차량의 현재 위치(current_node)에서부터의 전체 경로 총 누적 운행 시간 계산
            # IDLE_MOVE 경로가 있으면 배정 시에는 없는 것으로 간주함
            is_idle_move = len(vehicle.path) == 1 and vehicle.path[0].passenger_id == "IDLE_MOVE"
            eval_path = [] if is_idle_move else vehicle.path
            
            original_path_time = self._calculate_path_time(vehicle.current_node, eval_path)
            if math.isinf(original_path_time):
                # 현재 경로조차 유효하게 계산되지 않으면 해당 차량은 배제
                continue

            # 차량별 균등 샘플링: 모든 후보를 생성하고 샘플링
            candidate_path, candidate_cost, pickup_eta = self._find_best_insertion_full_v2(
                vehicle, request, original_path_time, eval_path
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

    def assign_street_hail_request(
        self,
        vehicles: Sequence[VehicleState],
        request: PassengerRequest,
        current_time: float = 0.0,
    ) -> Tuple[Optional[VehicleState], List[Stop], float]:
        """
        길거리 대기 승객(Street Hail) 전용 배정 로직.
        - 승객의 대기 위치(pickup_node)가 차량의 현재 운전 경로(모든 중간 노드 포함)상에 있을 때만 후보군에 포함.
        - 후보군 차량에 대해 드롭오프 삽입 위치만 전수 조사 평가.
        - 설정된 `STREET_HAIL_TRAVEL_TIME_INCREASE_LIMIT` 이내의 지연 시나리오만 허용.
        """
        best_vehicle: Optional[VehicleState] = None
        best_new_path: List[Stop] = []
        min_final_cost: float = math.inf

        assigned_time = current_time
        wait_assign = max(0.0, assigned_time - request.request_time)
        
        found_pickup_in_route = False
        found_exceeds_limit = False

        for vehicle in vehicles:
            if vehicle.onboard_passengers >= vehicle.capacity:
                continue

            if len(vehicle.path) > self.max_path_length:
                continue

            # IDLE_MOVE 경로가 있으면 배정 시에는 없는 것으로 간주함
            is_idle_move = len(vehicle.path) == 1 and vehicle.path[0].passenger_id == "IDLE_MOVE"
            eval_path = [] if is_idle_move else vehicle.path

            # 차량의 현재 위치(current_node)부터 남은 경로까지 거쳐가는 모든 노드를 시퀀스(경로)로 파악
            # (차항지 경유 포함, 전체 누적 운행 시간)
            original_path_time = self._calculate_path_time(vehicle.current_node, eval_path)
            if math.isinf(original_path_time):
                continue
            
            # 차량 출발지(current_node) 또는 차고지(depot)에서부터의 전체 경로 중
            # request.pickup_node 를 실제로 지나가는지 확인하고,
            # 지나갈 경우 그 픽업의 Stop index 위치를 반환
            pickup_index = self._find_pickup_index_in_route_v2(vehicle, request.pickup_node, eval_path)
            if pickup_index == -1:
                # 이 차량의 예정 경로에 이 승객의 픽업 위치가 없음
                continue
                
            found_pickup_in_route = True

            # 길거리 승객용 최적 드롭오프 위치 탐색
            candidate_path, candidate_cost, pickup_eta, cost_increase, exceeded_limit = self._find_best_insertion_street_hail(
                vehicle, request, original_path_time, pickup_index
            )
            if candidate_path is None:
                if exceeded_limit:
                    found_exceeds_limit = True
                continue

            # 가드레일: 도착 시간 증가가 LIMIT를 초과하면 거절
            if cost_increase > self.street_hail_travel_time_increase_limit:
                found_exceeds_limit = True
                continue

            if pickup_eta > self.max_dispatch_eta:
                continue

            wait_pickup = max(0.0, pickup_eta)
            total_wait = wait_assign + wait_pickup

            final_cost = candidate_cost + W_WAIT_TIME * total_wait

            if final_cost < min_final_cost:
                best_vehicle = vehicle
                best_new_path = candidate_path
                min_final_cost = final_cost

        if best_vehicle is None:
            if found_exceeds_limit:
                request.street_fail_reason = "과중한_경로_증가"
            return None, [], math.inf

        return best_vehicle, best_new_path, min_final_cost

    # 내부 메서드 ---------------------------------------------------------------------
    def _find_best_insertion_full_v2(
        self,
        vehicle: VehicleState,
        request: PassengerRequest,
        original_path_time: float,
        eval_path: List[Stop],
    ) -> Tuple[Optional[List[Stop]], float, float]:
        """
        주어진 차량 경로에서 픽업·드롭오프를 삽입할 최적 위치와 비용을 탐색 (전수 조사).
        eval_path를 명시적으로 전달받아 IDLE_MOVE를 배제한 경로 기준으로 계산함.
        """
        path_len = len(eval_path)
        
        # 성능 최적화: 경로 길이 제한
        if path_len + 2 > self.max_path_length:
            # 새 경로가 제한을 초과하면 배제
            return None, math.inf, math.inf

        # 성능 최적화: 원본 경로의 중간 노드 위치를 미리 계산 (증분 계산용)
        # 누적 운행 시간 계산을 위해 현재 위치(current_node)를 시작점으로 사용
        original_path_nodes = self._get_path_nodes(vehicle.current_node, eval_path)
        
        # 모든 가능한 후보 조합 생성
        all_candidates: List[Candidate] = []
        for pickup_index in range(path_len + 1):
            for dropoff_index in range(pickup_index + 1, path_len + 2):  # path_with_pickup 길이는 path_len + 1
                all_candidates.append(Candidate(pickup_index=pickup_index, dropoff_index=dropoff_index))
        
        # 전수 조사: 샘플링 없이 모든 후보 평가
        sampled_candidates = all_candidates
        
        # 모든 후보를 평가
        best_path: Optional[List[Stop]] = None
        best_partial_cost: float = math.inf   # w1*cost_increase + w2*new_path_time 항
        best_pickup_eta: float = math.inf
        
        for candidate in sampled_candidates:
            path_with_pickup = list(eval_path)
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

            # 성능 최적화: 증분 계산 사용
            if path_len <= 5:
                # 현재 위치(current_node)에서부터 전체 누적 운행 시간 다시 계산
                new_path_time = self._calculate_path_time(vehicle.current_node, path_candidate)
            else:
                # 증분 계산 사용 (eval_path 기준)
                new_path_time = self._calculate_path_time_incremental(
                    vehicle.current_node,
                    eval_path,
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
            # w1, w2 항만 계산
            partial_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_path_time)

            # 픽업 ETA: eval_path 기준 계산
            pickup_eta = self._calculate_pickup_eta(
                vehicle.current_node, eval_path, candidate.pickup_index, request.pickup_node
            )

            # 성능 최적화: 조기 종료
            if best_partial_cost != math.inf and partial_cost > best_partial_cost * EARLY_TERMINATION_THRESHOLD:
                continue

            if partial_cost < best_partial_cost:
                best_partial_cost = partial_cost
                best_path = path_candidate
                best_pickup_eta = pickup_eta

        return best_path, best_partial_cost, best_pickup_eta

    def _find_best_insertion_full(
        self,
        vehicle: VehicleState,
        request: PassengerRequest,
        original_path_time: float,
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
        # 누적 운행 시간 계산을 위해 현재 위치(current_node)를 시작점으로 사용
        original_path_nodes = self._get_path_nodes(vehicle.current_node, vehicle.path)
        
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
                # 현재 위치(current_node)에서부터 전체 누적 운행 시간 다시 계산
                new_path_time = self._calculate_path_time(vehicle.current_node, path_candidate)
            else:
                # 경로가 길면 증분 계산 사용 (현재 위치 출발 기준)
                new_path_time = self._calculate_path_time_incremental(
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
            # w1, w2 항만 계산 (w3 항은 assign_request에서 추가)
            partial_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_path_time)

            # 픽업 ETA: 현재 위치(current_node) → 픽업까지 경유하는 스톱들을 따라 이동한 시간
            pickup_eta = self._calculate_pickup_eta(
                vehicle.current_node, vehicle.path, candidate.pickup_index, request.pickup_node
            )

            # 성능 최적화: 조기 종료 - 현재 최적해보다 훨씬 나쁘면 건너뛰기
            if best_partial_cost != math.inf and partial_cost > best_partial_cost * EARLY_TERMINATION_THRESHOLD:
                continue

            if partial_cost < best_partial_cost:
                best_partial_cost = partial_cost
                best_path = path_candidate
                best_pickup_eta = pickup_eta

        return best_path, best_partial_cost, best_pickup_eta

    def _find_best_insertion_street_hail(
        self,
        vehicle: VehicleState,
        request: PassengerRequest,
        original_path_time: float,
        fixed_pickup_index: int,
    ) -> Tuple[Optional[List[Stop]], float, float, float, bool]:
        """
        길거리 승객 전용 삽입 전수조사. (픽업 인덱스가 고정되어 있음)
        반환값: (최적 경로, 부분비용, 픽업ETA, cost_increase, limit_exceeded_only)
        """
        path_len = len(vehicle.path)
        if path_len + 2 > self.max_path_length:
            return None, math.inf, math.inf, math.inf, False

        original_path_nodes = self._get_path_nodes(vehicle.current_node, vehicle.path)

        best_path: Optional[List[Stop]] = None
        best_partial_cost: float = math.inf
        best_pickup_eta: float = math.inf
        min_cost_increase: float = math.inf
        any_exceeded_limit = False
        valid_insertion_found = False

        # 드롭오프는 픽업 후 어디든 가능
        for dropoff_index in range(fixed_pickup_index + 1, path_len + 2):
            path_with_pickup = vehicle.clone_path()
            path_with_pickup.insert(
                fixed_pickup_index,
                Stop(node_id=request.pickup_node, stop_type="pickup", passenger_id=request.passenger_id, is_street_hail=True),
            )
            
            path_candidate = list(path_with_pickup)
            path_candidate.insert(
                dropoff_index,
                Stop(node_id=request.dropoff_node, stop_type="dropoff", passenger_id=request.passenger_id, is_street_hail=True),
            )

            if not self._is_capacity_valid(path_candidate, vehicle.capacity, vehicle.onboard_passengers):
                continue

            if path_len <= 5:
                new_path_time = self._calculate_path_time(vehicle.current_node, path_candidate)
            else:
                new_path_time = self._calculate_path_time_incremental(
                    vehicle.current_node, vehicle.path, original_path_nodes, original_path_time,
                    fixed_pickup_index, request.pickup_node, dropoff_index, request.dropoff_node
                )

            if math.isinf(new_path_time):
                continue

            cost_increase = new_path_time - original_path_time
            
            if cost_increase > self.street_hail_travel_time_increase_limit:
                any_exceeded_limit = True
                continue
                
            valid_insertion_found = True

            partial_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_path_time)

            pickup_eta = self._calculate_pickup_eta(
                vehicle.current_node, vehicle.path, fixed_pickup_index, request.pickup_node
            )

            if best_partial_cost != math.inf and partial_cost > best_partial_cost * EARLY_TERMINATION_THRESHOLD:
                continue

            if partial_cost < best_partial_cost:
                best_partial_cost = partial_cost
                best_path = path_candidate
                best_pickup_eta = pickup_eta
                min_cost_increase = cost_increase

        return best_path, best_partial_cost, best_pickup_eta, min_cost_increase, (any_exceeded_limit and not valid_insertion_found)

    def _find_pickup_index_in_route_v2(self, vehicle: VehicleState, pickup_node: int, eval_path: List[Stop]) -> int:
        """
        eval_path를 기준으로 픽업 노드가 길목에 있는지 확인.
        """
        current_node = vehicle.current_node

        # 남은 Stop이 하나도 없는(비어있는) 경우
        if not eval_path:
            # 차량이 이미 pickup_node에 있으면 가장 앞에 삽입(0)
            if current_node == pickup_node:
                return 0
            return -1

        # 현재 위치와 첫 번째 Stop 사이 경로 확인
        try:
            route = nx.shortest_path(self.graph, current_node, eval_path[0].node_id, weight="weight")
            if pickup_node in route:
                return 0
        except nx.NetworkXNoPath:
            pass

        # 중간 Stop들 사이 경로 확인
        for i in range(len(eval_path) - 1):
            start = eval_path[i].node_id
            end = eval_path[i+1].node_id
            try:
                route = nx.shortest_path(self.graph, start, end, weight="weight")
                if pickup_node in route:
                    return i + 1
            except nx.NetworkXNoPath:
                continue

        return -1

    def _find_pickup_index_in_route(self, vehicle: VehicleState, pickup_node: int) -> int:
        """
        차량의 현재 노드 ~ 모든 남은 Stop 경로 사이의 세부 노드를 탐색하여,
        주어진 pickup_node가 지나가는 길목에 포함되어 있으면, 
        어느 Stop 순서에 픽업을 삽입해야 하는지(index)를 반환. 없으면 -1 반환.
        """
        current_node = vehicle.current_node

        # 남은 Stop이 하나도 없는(비어있는) 경우
        if not vehicle.path:
            # 차량이 이미 pickup_node에 있으면 가장 앞에 삽입(0)
            if current_node == pickup_node:
                return 0
            return -1

        # 현재 위치와 첫 번째 Stop 사이 경로 확인
        try:
            # current_node -> path[0].node_id 로 가는 경로 노드들
            route = nx.shortest_path(self.graph, current_node, vehicle.path[0].node_id, weight="weight")
            if pickup_node in route:
                return 0 # 첫 번째 Stop 이전에 삽입
        except nx.NetworkXNoPath:
            pass

        # 중간 Stop들 사이 경로 확인
        for i in range(len(vehicle.path) - 1):
            start = vehicle.path[i].node_id
            end = vehicle.path[i+1].node_id
            try:
                route = nx.shortest_path(self.graph, start, end, weight="weight")
                # start 노드는 바로 앞 세그먼트의 끝 혹은 current_node의 출발지였으므로
                # 중복이지만 포함 여부만 확인하므로 ok.
                if pickup_node in route:
                    return i + 1 # i번째 Stop 이후, 즉 index로는 i+1 자리
            except nx.NetworkXNoPath:
                continue

        return -1

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