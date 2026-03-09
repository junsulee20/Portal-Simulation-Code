"""
Wait-time 가중치 Grid Search 시뮬레이션.

원본: drt_network_simulation_realized.py 에서 최소 수정.

실행 한 번으로 다음을 수행합니다:
    1. Baseline (w1=0.7, w2=0.3, w3=0.0) 시뮬레이션 실행
    2. Grid Search: w1 ∈ {0.5,0.6,0.7,0.8}, w2 ∈ {0.1,0.2,0.3,0.4},
                   w3 = 1 - w1 - w2  (w3 < 0인 조합 제외)
    3. 각 조합의 p50/p95/p99/max/mean 통계 계산
    4. 가드레일 적용: cost_increase p95 or new_path_time p95가 Baseline 대비 +10% 초과 시 제외
    5. total_wait p95 기준 정렬 → 최적 가중치 출력
    6. 결과를 CSV 파일로 저장

모든 단위는 초(seconds).
"""

from __future__ import annotations

import csv
import math
import random
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")   # 그래프를 파일로 저장할 때 GUI 불필요
import matplotlib.pyplot as plt
from matplotlib import font_manager
import networkx as nx

# ─── 새로운 assignment 모듈 임포트 ──────────────────────────────────────────────────
import drt_network_assignment_wait_time as _asgn
from drt_network_assignment_wait_time import (
    DRTAssignmentEngine,
    NetworkTravelTimeCache,
    PassengerRequest,
    Stop,
    VehicleState,
    load_network_graph,
    ENABLE_WAITING_COST,
)

# =====================================================================================
# 시뮬레이션 설정
# =====================================================================================
NUM_DEMANDS = 120
NUM_VEHICLES = 5
VEHICLE_CAPACITY = 14
REQUEST_INTERVAL_SECONDS = 30

DEMAND_SEED = 42
VEHICLE_SEED = 123

REGION_MIN_LAT = 37.107927
REGION_MAX_LAT = 37.140525
REGION_MIN_LON = 126.903436
REGION_MAX_LON = 126.929743

# ─── Grid 탐색 설정 ───────────────────────────────────────────────────────────────────
BASELINE_W = (0.7, 0.3, 0.0)

W1_GRID = [0.5, 0.6, 0.7, 0.8]
W2_GRID = [0.1, 0.2, 0.3, 0.4]

# 가드레일 허용 초과 비율 (baseline 대비)
GUARDRAIL_RATIO = 0.10  # 10%

# 결과 CSV 저장 경로 (실행 디렉토리 기준)
RESULT_CSV_PATH = "wait_time_grid_search_results_120.csv"

# =====================================================================================
# Matplotlib 한글 폰트 설정
# =====================================================================================


def configure_matplotlib_font() -> None:
    preferred_fonts = [
        "Malgun Gothic", "NanumGothic", "Nanum Gothic",
        "AppleGothic", "NanumGothicCoding", "Noto Sans CJK KR", "Noto Sans KR",
    ]
    available_fonts = {f.name for f in font_manager.fontManager.ttflist}
    for font_name in preferred_fonts:
        if font_name in available_fonts:
            plt.rcParams["font.family"] = font_name
            break
    plt.rcParams["axes.unicode_minus"] = False


configure_matplotlib_font()

PRESET_VEHICLE_COLORS: Dict[int, str] = {1: "#0077b6", 2: "#ef476f"}
FALLBACK_VEHICLE_COLORS: List[str] = [
    "#2a9d8f", "#f4a261", "#e76f51", "#8338ec", "#06d6a0", "#ffafcc",
]

# =====================================================================================
# 데이터 구조
# =====================================================================================


@dataclass
class AssignmentEvent:
    """단일 배차 이벤트 기록 — wait_assign / wait_pickup / total_wait / cost_increase 추가."""

    request: PassengerRequest
    vehicle_id: int
    vehicle_start_node: int
    cost: float
    previous_path: List[Stop]
    new_path: List[Stop]
    request_time: float
    assignment_time: float
    # ── 기존 필드 ──────────────────────────
    waiting_time: float        # wait_assign (요청→배정, 현재 구현에선 ≈ 0)
    travel_time: float         # 픽업→드롭오프 시간
    pickup_time: float
    dropoff_time: float
    straight_line_distance_degrees: float
    straight_line_distance_km: float
    speed_kmh: float
    # ── 신규 추가 필드 ─────────────────────
    wait_assign: float         # 요청 ~ 배정 (초)
    wait_pickup: float         # 배정 ~ 픽업 ETA (초)
    total_wait: float          # wait_assign + wait_pickup (초)
    cost_increase: float       # new_path_time - original_path_time (초)
    new_path_time: float       # 새 경로의 전체 이동 시간 (초)


@dataclass
class FixedDemand:
    passenger_id: str
    pickup_node: int
    dropoff_node: int


@dataclass
class FixedVehicleInitial:
    vehicle_id: int
    initial_node: int


# =====================================================================================
# 지역 필터링 / 고정 디멘드·차량 생성 (원본과 동일)
# =====================================================================================


def filter_nodes_by_region(
    graph: nx.Graph,
    min_lon: float, max_lon: float,
    min_lat: float, max_lat: float,
) -> List[int]:
    filtered_nodes = []
    for node in graph.nodes:
        node_data = graph.nodes.get(node)
        if not node_data:
            continue
        lon = node_data.get("longitude")
        lat = node_data.get("latitude")
        if lon is None or lat is None:
            continue
        if min_lon <= lon <= max_lon and min_lat <= lat <= max_lat:
            filtered_nodes.append(node)
    return filtered_nodes


def generate_fixed_demands(
    graph: nx.Graph, num_demands: int, seed: int,
    allowed_nodes: Optional[List[int]] = None,
) -> List[FixedDemand]:
    random_gen = random.Random(seed)
    nodes = allowed_nodes if allowed_nodes is not None else list(graph.nodes)
    if not nodes:
        raise ValueError("허용된 노드가 없습니다.")
    demands = []
    for idx in range(1, num_demands + 1):
        pickup = random_gen.choice(nodes)
        dropoff = random_gen.choice(nodes)
        while dropoff == pickup:
            dropoff = random_gen.choice(nodes)
        demands.append(FixedDemand(
            passenger_id=f"demand_{idx:03d}",
            pickup_node=pickup,
            dropoff_node=dropoff,
        ))
    return demands


def generate_fixed_vehicle_initials(
    num_vehicles: int, graph: nx.Graph, seed: int,
    allowed_nodes: Optional[List[int]] = None,
) -> List[FixedVehicleInitial]:
    random_gen = random.Random(seed)
    nodes = allowed_nodes if allowed_nodes is not None else list(graph.nodes)
    if not nodes:
        raise ValueError("허용된 노드가 없습니다.")
    depot_node = random_gen.choice(nodes)
    return [
        FixedVehicleInitial(vehicle_id=idx, initial_node=depot_node)
        for idx in range(1, num_vehicles + 1)
    ]


# =====================================================================================
# 헬퍼 함수
# =====================================================================================


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
    graph: nx.Graph, node1: int, node2: int,
) -> Tuple[float, float]:
    lon1, lat1 = node_lonlat(graph, node1)
    lon2, lat2 = node_lonlat(graph, node2)
    if None in (lon1, lat1, lon2, lat2):
        return 0.0, 0.0
    dx, dy = lon2 - lon1, lat2 - lat1
    distance_degrees = math.sqrt(dx * dx + dy * dy)
    R = 6371.0
    lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
    dlat_rad, dlon_rad = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dlat_rad / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2) ** 2
    distance_km = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return distance_degrees, distance_km


def calculate_path_travel_time(
    travel_time_cache: NetworkTravelTimeCache,
    start_node: int,
    path: List[Stop],
    pickup_node: int,
    dropoff_node: int,
) -> Tuple[float, float, float]:
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
    return dropoff_time - pickup_time, pickup_time, dropoff_time


def build_route_nodes(graph: nx.Graph, start_node: int, stops: Iterable[Stop]) -> List[int]:
    route: List[int] = [start_node]
    current = start_node
    for stop in stops:
        try:
            segment = nx.shortest_path(graph, current, stop.node_id, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            segment = [current, stop.node_id]
        route.extend(segment[1:])
        current = stop.node_id
    return route


# =====================================================================================
# 통계 계산 유틸
# =====================================================================================


def percentile(data: List[float], pct: float) -> float:
    """data 리스트에서 pct 백분위수를 반환 (선형 보간)."""
    if not data:
        return math.nan
    sorted_data = sorted(data)
    n = len(sorted_data)
    idx = (n - 1) * pct / 100.0
    lo, hi = int(idx), min(int(idx) + 1, n - 1)
    if lo == hi:
        return sorted_data[lo]
    return sorted_data[lo] + (sorted_data[hi] - sorted_data[lo]) * (idx - lo)


def compute_stats(values: List[float]) -> Dict[str, float]:
    """p50/p95/p99/max/mean 통계 딕셔너리 반환."""
    valid = [v for v in values if not math.isnan(v) and not math.isinf(v)]
    if not valid:
        return {"mean": math.nan, "p50": math.nan, "p95": math.nan, "p99": math.nan, "max": math.nan}
    return {
        "mean": statistics.mean(valid),
        "p50":  percentile(valid, 50),
        "p95":  percentile(valid, 95),
        "p99":  percentile(valid, 99),
        "max":  max(valid),
    }


# =====================================================================================
# 단일 시뮬레이션 실행 (가중치 파라미터 주입)
# =====================================================================================


def run_one_simulation(
    graph: nx.Graph,
    allowed_nodes: List[int],
    fixed_demands: List[FixedDemand],
    vehicle_initials: List[FixedVehicleInitial],
    w1: float,
    w2: float,
    w3: float,
    verbose: bool = False,
) -> List[AssignmentEvent]:
    """
    하나의 (w1, w2, w3) 조합으로 시뮬레이션 실행 후 이벤트 목록 반환.
    seed는 fixed_demands와 vehicle_initials를 통해 고정됩니다.
    """
    # 새 엔진 생성 (가중치 주입)
    engine = DRTAssignmentEngine(graph, w1=w1, w2=w2, w3=w3)

    # 차량 상태 초기화
    vehicles: List[VehicleState] = [
        VehicleState(
            vehicle_id=init.vehicle_id,
            current_node=init.initial_node,
            capacity=VEHICLE_CAPACITY,
            onboard_passengers=0,
        )
        for init in vehicle_initials
    ]

    events: List[AssignmentEvent] = []
    current_time = 0.0

    for demand in fixed_demands:
        request = PassengerRequest(
            passenger_id=demand.passenger_id,
            pickup_node=demand.pickup_node,
            dropoff_node=demand.dropoff_node,
        )

        # wait_assign: 이 구현에서는 배정이 즉시 이루어지므로 0
        wait_assign = 0.0
        assignment_time = current_time

        # assign_request → (vehicle, new_path, cost, wait_pickup)
        assigned_vehicle, new_path, cost, wait_pickup = engine.assign_request(
            vehicles, request, wait_assign_seconds=wait_assign
        )

        if assigned_vehicle is None:
            if verbose:
                print(f"  [배차 실패] {request.passenger_id}")
            current_time += REQUEST_INTERVAL_SECONDS
            continue

        previous_path = assigned_vehicle.clone_path()
        start_node = assigned_vehicle.current_node

        # 기존 경로 이동 시간 (cost_increase 계산용)
        original_path_time = engine._calculate_path_time(start_node, previous_path)
        new_path_time_val = engine._calculate_path_time(start_node, new_path)
        cost_increase_val = new_path_time_val - original_path_time

        # 통행시간 계산 (픽업→드롭오프)
        travel_time, pickup_time_from_start, dropoff_time_from_start = calculate_path_travel_time(
            engine.travel_time_cache, start_node, new_path,
            request.pickup_node, request.dropoff_node,
        )

        pickup_time = assignment_time + pickup_time_from_start
        dropoff_time = assignment_time + dropoff_time_from_start
        waiting_time = pickup_time - current_time  # wait_assign + wait_pickup (총 대기)

        total_wait = wait_assign + (wait_pickup if not math.isinf(wait_pickup) else pickup_time_from_start)

        dist_deg, dist_km = calculate_straight_line_distance(
            graph, request.pickup_node, request.dropoff_node
        )
        speed_kmh = (dist_km / travel_time * 3600) if travel_time > 0 and not math.isinf(travel_time) else 0.0

        event = AssignmentEvent(
            request=request,
            vehicle_id=assigned_vehicle.vehicle_id,
            vehicle_start_node=start_node,
            cost=cost,
            previous_path=previous_path,
            new_path=list(new_path),
            request_time=current_time,
            assignment_time=assignment_time,
            waiting_time=waiting_time,
            travel_time=travel_time,
            pickup_time=pickup_time,
            dropoff_time=dropoff_time,
            straight_line_distance_degrees=dist_deg,
            straight_line_distance_km=dist_km,
            speed_kmh=speed_kmh,
            wait_assign=wait_assign,
            wait_pickup=wait_pickup if not math.isinf(wait_pickup) else pickup_time_from_start,
            total_wait=total_wait,
            cost_increase=cost_increase_val,
            new_path_time=new_path_time_val,
        )

        assigned_vehicle.path = new_path
        events.append(event)

        if verbose:
            print(
                f"  [{request.passenger_id}] 차량{assigned_vehicle.vehicle_id} | "
                f"wait_assign={wait_assign:.0f}s wait_pickup={event.wait_pickup:.0f}s "
                f"total_wait={total_wait:.0f}s cost_inc={cost_increase_val:.0f}s"
            )

        current_time += REQUEST_INTERVAL_SECONDS

    return events


# =====================================================================================
# Grid Search + 통계 집계
# =====================================================================================


def build_weight_grid() -> List[Tuple[float, float, float]]:
    """
    w1 ∈ W1_GRID, w2 ∈ W2_GRID, w3 = 1 - w1 - w2 (w3 >= 0 만) 조합 반환.
    baseline은 맨 앞에 별도 추가됩니다.
    """
    combos: List[Tuple[float, float, float]] = []
    seen = set()

    # Baseline 먼저
    combos.append(BASELINE_W)
    seen.add(BASELINE_W)

    for w1 in W1_GRID:
        for w2 in W2_GRID:
            w3 = round(1.0 - w1 - w2, 6)
            if w3 < 0:
                continue
            triple = (round(w1, 3), round(w2, 3), round(w3, 3))
            if triple not in seen:
                seen.add(triple)
                combos.append(triple)

    return combos


def summarize_events(events: List[AssignmentEvent]) -> Dict[str, Dict[str, float]]:
    """이벤트 목록에서 total_wait / wait_assign / wait_pickup / cost_increase / new_path_time 통계 반환."""
    return {
        "total_wait":    compute_stats([e.total_wait    for e in events]),
        "wait_assign":   compute_stats([e.wait_assign   for e in events]),  # 요청→배정
        "wait_pickup":   compute_stats([e.wait_pickup   for e in events]),  # 배정→픽업 ETA
        "cost_increase": compute_stats([e.cost_increase for e in events]),
        "new_path_time": compute_stats([e.new_path_time for e in events]),
        "travel_time":   compute_stats([e.travel_time   for e in events
                                        if not math.isinf(e.travel_time)]),
    }


def run_grid_search(
    graph: nx.Graph,
    allowed_nodes: List[int],
    fixed_demands: List[FixedDemand],
    vehicle_initials: List[FixedVehicleInitial],
) -> List[Dict]:
    """
    모든 가중치 조합을 순서대로 실행하고, 조합별 통계를 포함한 결과 딕셔너리 목록 반환.
    결과 목록의 첫 번째 항목이 항상 baseline입니다.
    """
    weight_grid = build_weight_grid()
    print(f"\n{'='*70}")
    print(f"  Grid Search 시작: 총 {len(weight_grid)}개 조합 (Baseline 포함)")
    print(f"  ENABLE_WAITING_COST = {ENABLE_WAITING_COST}")
    print(f"{'='*70}\n")

    results = []
    for i, (w1, w2, w3) in enumerate(weight_grid, 1):
        label = "BASELINE" if (w1, w2, w3) == BASELINE_W else f"({w1},{w2},{w3})"
        print(f"[{i:2d}/{len(weight_grid)}] w=({w1:.1f},{w2:.1f},{w3:.2f}) ... ", end="", flush=True)
        t0 = time.perf_counter()
        events = run_one_simulation(
            graph, allowed_nodes, fixed_demands, vehicle_initials,
            w1=w1, w2=w2, w3=w3,
        )
        elapsed = time.perf_counter() - t0
        stats = summarize_events(events)
        print(f"완료 ({elapsed:.2f}s) → "
              f"total_wait p95={stats['total_wait']['p95']:.0f}s  "
              f"n={len(events)}")
        results.append({
            "w1": w1, "w2": w2, "w3": w3,
            "label": label,
            "n_success": len(events),
            "stats": stats,
            "elapsed": elapsed,
        })

    return results


# =====================================================================================
# 결과 출력 및 최적 가중치 선택
# =====================================================================================


def print_results_table(results: List[Dict], baseline_stats: Dict[str, Dict[str, float]]) -> None:
    """조합별 통계를 표 형태로 출력, 가드레일 통과 여부를 함께 표시."""
    guard_ci_p95 = baseline_stats["cost_increase"]["p95"] * (1 + GUARDRAIL_RATIO)
    guard_np_p95 = baseline_stats["new_path_time"]["p95"] * (1 + GUARDRAIL_RATIO)

    header = (
        f"{'No':>3}  {'w1':>4} {'w2':>4} {'w3':>5}  "
        f"{'tw_mean':>8} {'tw_p50':>8} {'tw_p95':>8} {'tw_p99':>8}  "
        f"{'ci_p95':>8} {'np_p95':>8}  {'Guard':>7}  {'n':>3}"
    )
    print("\n" + "=" * len(header))
    print(header)
    print("=" * len(header))

    for i, r in enumerate(results, 1):
        s = r["stats"]
        tw_p95 = s["total_wait"]["p95"]
        ci_p95 = s["cost_increase"]["p95"]
        np_p95 = s["new_path_time"]["p95"]
        guard_ok = (ci_p95 <= guard_ci_p95) and (np_p95 <= guard_np_p95)
        guard_str = "✅ OK" if guard_ok else "❌ NG"
        mark = " ← BASELINE" if r["label"] == "BASELINE" else ""
        print(
            f"{i:>3}  {r['w1']:>4.2f} {r['w2']:>4.2f} {r['w3']:>5.3f}  "
            f"{s['total_wait']['mean']:>8.0f} {s['total_wait']['p50']:>8.0f} "
            f"{tw_p95:>8.0f} {s['total_wait']['p99']:>8.0f}  "
            f"{ci_p95:>8.0f} {np_p95:>8.0f}  "
            f"{guard_str}  {r['n_success']:>3}{mark}"
        )
    print("=" * len(header))


def select_best(results: List[Dict], baseline_stats: Dict[str, Dict[str, float]]) -> Optional[Dict]:
    """가드레일을 통과한 조합 중 total_wait p95가 가장 낮은 조합 반환."""
    guard_ci_p95 = baseline_stats["cost_increase"]["p95"] * (1 + GUARDRAIL_RATIO)
    guard_np_p95 = baseline_stats["new_path_time"]["p95"] * (1 + GUARDRAIL_RATIO)

    valid = [
        r for r in results
        if (r["stats"]["cost_increase"]["p95"] <= guard_ci_p95 and
            r["stats"]["new_path_time"]["p95"] <= guard_np_p95)
    ]
    if not valid:
        return None

    # 1차: total_wait p95 최소 / 2차(타이브레이커): total_wait mean 최소
    valid.sort(key=lambda r: (r["stats"]["total_wait"]["p95"], r["stats"]["total_wait"]["mean"]))
    return valid[0]


def save_csv(results: List[Dict], baseline_stats: Dict[str, Dict[str, float]], path: str) -> None:
    """결과를 CSV 파일로 저장.

    컬럼 설명:
        tw_*  : total_wait  (wait_assign + wait_pickup) [s]
        wa_*  : wait_assign (요청 수신 → 배정 완료)     [s]  ← 신규
        wp_*  : wait_pickup (배정 완료 → 픽업 ETA)      [s]  ← 신규
        ci_*  : cost_increase (신규 경로 - 기존 경로 시간) [s]
        np_*  : new_path_time (배정 후 전체 경로 시간)   [s]
        tt_*  : travel_time (픽업 → 드롭오프)            [s]
    """
    guard_ci_p95 = baseline_stats["cost_increase"]["p95"] * (1 + GUARDRAIL_RATIO)
    guard_np_p95 = baseline_stats["new_path_time"]["p95"] * (1 + GUARDRAIL_RATIO)

    fieldnames = [
        "no", "w1", "w2", "w3", "label", "n_success",
        # ── 총 대기 ──────────────────────────────────────────────
        "tw_mean", "tw_p50", "tw_p95", "tw_p99", "tw_max",
        # ── 대기 구간 1: 요청→배정 ───────────────────────────────
        "wa_mean", "wa_p50", "wa_p95", "wa_max",
        # ── 대기 구간 2: 배정→픽업 ETA ──────────────────────────
        "wp_mean", "wp_p50", "wp_p95", "wp_max",
        # ── 기존 지표 ────────────────────────────────────────────
        "ci_mean", "ci_p95",
        "np_mean", "np_p95",
        "tt_mean", "tt_p95",
        "guard_pass", "elapsed_s",
    ]

    with open(path, "w", newline="", encoding="utf-8-sig") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for i, r in enumerate(results, 1):
            s = r["stats"]
            guard_pass = (
                s["cost_increase"]["p95"] <= guard_ci_p95 and
                s["new_path_time"]["p95"] <= guard_np_p95
            )
            writer.writerow({
                "no":          i,
                "w1":          r["w1"],
                "w2":          r["w2"],
                "w3":          r["w3"],
                "label":       r["label"],
                "n_success":   r["n_success"],
                # total_wait
                "tw_mean":     round(s["total_wait"]["mean"],    2),
                "tw_p50":      round(s["total_wait"]["p50"],     2),
                "tw_p95":      round(s["total_wait"]["p95"],     2),
                "tw_p99":      round(s["total_wait"]["p99"],     2),
                "tw_max":      round(s["total_wait"]["max"],     2),
                # wait_assign (요청→배정)
                "wa_mean":     round(s["wait_assign"]["mean"],   2),
                "wa_p50":      round(s["wait_assign"]["p50"],    2),
                "wa_p95":      round(s["wait_assign"]["p95"],    2),
                "wa_max":      round(s["wait_assign"]["max"],    2),
                # wait_pickup (배정→픽업)
                "wp_mean":     round(s["wait_pickup"]["mean"],   2),
                "wp_p50":      round(s["wait_pickup"]["p50"],    2),
                "wp_p95":      round(s["wait_pickup"]["p95"],    2),
                "wp_max":      round(s["wait_pickup"]["max"],    2),
                # cost / path
                "ci_mean":     round(s["cost_increase"]["mean"], 2),
                "ci_p95":      round(s["cost_increase"]["p95"],  2),
                "np_mean":     round(s["new_path_time"]["mean"], 2),
                "np_p95":      round(s["new_path_time"]["p95"],  2),
                # travel
                "tt_mean":     round(s["travel_time"]["mean"],   2),
                "tt_p95":      round(s["travel_time"]["p95"],    2),
                "guard_pass":  "Y" if guard_pass else "N",
                "elapsed_s":   round(r["elapsed"],               3),
            })
    print(f"\n💾  결과 CSV 저장 완료: {path}\n")


# =====================================================================================
# 메인 실행
# =====================================================================================


def main() -> None:
    print("=" * 70)
    print("화성똑버스03 DRT — Wait Time 가중치 Grid Search")
    print(f"  ENABLE_WAITING_COST = {ENABLE_WAITING_COST}")
    print("=" * 70)
    print()

    print(f"[설정] 디멘드: {NUM_DEMANDS}개 | 차량: {NUM_VEHICLES}대 | 용량: {VEHICLE_CAPACITY}명")
    print(f"[설정] 요청 간격: {REQUEST_INTERVAL_SECONDS}초 | seed demand={DEMAND_SEED}, vehicle={VEHICLE_SEED}")
    print()

    # ── 그래프 로드 ──────────────────────────────────────────────────────────────────
    graph = load_network_graph()

    # ── 지역 필터링 ──────────────────────────────────────────────────────────────────
    allowed_nodes = filter_nodes_by_region(
        graph, REGION_MIN_LON, REGION_MAX_LON, REGION_MIN_LAT, REGION_MAX_LAT
    )
    if not allowed_nodes:
        raise ValueError("지정된 좌표 범위 내에 노드가 없습니다.")
    print(f"[지역 필터링] 향남신도시 — 필터링된 노드 수: {len(allowed_nodes)}개\n")

    # ── seed 고정: 디멘드·차량 초기 위치 생성 ─────────────────────────────────────
    fixed_demands = generate_fixed_demands(graph, NUM_DEMANDS, DEMAND_SEED, allowed_nodes)
    vehicle_initials = generate_fixed_vehicle_initials(NUM_VEHICLES, graph, VEHICLE_SEED, allowed_nodes)
    print(f"[디멘드 생성] 총 {len(fixed_demands)}개 | seed={DEMAND_SEED}")
    print(f"[차량 초기위치] 차고지 노드={vehicle_initials[0].initial_node}\n")

    # ── Grid Search 실행 ─────────────────────────────────────────────────────────────
    results = run_grid_search(graph, allowed_nodes, fixed_demands, vehicle_initials)

    # baseline 통계 (결과 첫 번째 항목)
    baseline_stats = results[0]["stats"]

    # ── 결과 표 출력 ─────────────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("📊  Grid Search 결과 요약")
    print(f"  가드레일 기준: cost_increase p95 ≤ {baseline_stats['cost_increase']['p95']*(1+GUARDRAIL_RATIO):.0f}s  "
          f"| new_path_time p95 ≤ {baseline_stats['new_path_time']['p95']*(1+GUARDRAIL_RATIO):.0f}s")
    print_results_table(results, baseline_stats)

    # ── CSV 저장 ─────────────────────────────────────────────────────────────────────
    save_csv(results, baseline_stats, RESULT_CSV_PATH)

    # ── 최적 가중치 출력 ─────────────────────────────────────────────────────────────
    best = select_best(results, baseline_stats)
    print("=" * 70)
    if best is None:
        print("⚠️  가드레일을 통과한 조합이 없습니다. GUARDRAIL_RATIO 완화를 고려하세요.")
    else:
        bs = best["stats"]
        baseline_tw_p95 = baseline_stats["total_wait"]["p95"]
        improvement = (baseline_tw_p95 - bs["total_wait"]["p95"]) / baseline_tw_p95 * 100 if baseline_tw_p95 > 0 else 0
        print(f"🏆  최적 가중치: w1={best['w1']}, w2={best['w2']}, w3={best['w3']}")
        print(f"    total_wait p95  : {bs['total_wait']['p95']:.0f}s  "
              f"(baseline {baseline_tw_p95:.0f}s, 개선율 {improvement:.1f}%)")
        print(f"    total_wait mean : {bs['total_wait']['mean']:.0f}s")
        print(f"    cost_increase p95: {bs['cost_increase']['p95']:.0f}s")
        print(f"    new_path_time p95: {bs['new_path_time']['p95']:.0f}s")
        print(f"    배정 성공 수       : {best['n_success']}")
    print("=" * 70)
    print()
    print("※ 상세 통계는 위에서 저장된 CSV 파일을 참고하세요.")
    print("  (tw=total_wait, ci=cost_increase, np=new_path_time, tt=travel_time)")


if __name__ == "__main__":
    main()
