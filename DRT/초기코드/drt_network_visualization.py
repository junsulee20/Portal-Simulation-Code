"""
실제 네트워크 그래프(`main_network_graph.pkl`)를 활용해
1:다수 DRT 배정 결과를 시각화하는 유틸리티 모듈.

`drt_network_assignment.py`의 배차 로직을 사용하며,
각 요청에 대해
  - 픽업/드롭오프 위치
  - 배정된 차량의 기존 경로와 업데이트된 경로
를 지도 형태(경도-위도 좌표)에 표시합니다.

기본 실행 예:
    python drt_network_visualization.py --requests 5

옵션:
    --requests N : 데모용 무작위 요청 개수 (기본 5)
    --save PATH  : 시각화 결과를 PNG로 저장 (기본 화면 출력)
"""

from __future__ import annotations

import argparse
import random
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib import font_manager
import networkx as nx
import numpy as np

from drt_network_assignment import (
    DRTAssignmentEngine,
    PassengerRequest,
    Stop,
    VehicleState,
    load_network_graph,
    select_random_node,
)

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


# 모듈 초기화 시 폰트 설정 시도
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


# --------------------------------------------------------------------------------------
# 시각화 루틴
# --------------------------------------------------------------------------------------


def plot_assignment_event(ax: plt.Axes, event: AssignmentEvent, graph: nx.Graph, color: str) -> None:
    """단일 배차 이벤트를 지도 위에 표현."""
    # 참고용 간선: 이전/이후 경로에 등장하는 노드 기반 서브그래프
    node_scope = (
        set(event.previous_route_nodes)
        | set(event.new_route_nodes)
        | {event.request.pickup_node, event.request.dropoff_node, event.vehicle_start_node}
    )
    subgraph = graph.subgraph(node_scope)

    for u, v in subgraph.edges():
        x1, y1 = node_lonlat(graph, u)
        x2, y2 = node_lonlat(graph, v)
        if None in (x1, y1, x2, y2):
            continue
        ax.plot([x1, x2], [y1, y2], color="#d0d0d0", linewidth=0.5, zorder=1)

    prev_xs, prev_ys = route_to_xy(graph, event.previous_route_nodes)
    if len(prev_xs) >= 2:
        ax.plot(prev_xs, prev_ys, linestyle="--", color="#8d99ae", linewidth=1.5, label="배차 전 경로", zorder=2)

    new_xs, new_ys = route_to_xy(graph, event.new_route_nodes)
    if len(new_xs) >= 2:
        ax.plot(new_xs, new_ys, linestyle="-", color=color, linewidth=2.4, label="배차 후 경로", zorder=3)

    pickup_xy = node_lonlat(graph, event.request.pickup_node)
    dropoff_xy = node_lonlat(graph, event.request.dropoff_node)
    start_xy = node_lonlat(graph, event.vehicle_start_node)

    if pickup_xy[0] is not None:
        ax.scatter(
            pickup_xy[0],
            pickup_xy[1],
            marker="*",
            s=160,
            color="#ffb703",
            edgecolor="k",
            linewidth=0.6,
            label="픽업 위치",
            zorder=4,
        )
    if dropoff_xy[0] is not None:
        ax.scatter(
            dropoff_xy[0],
            dropoff_xy[1],
            marker="X",
            s=110,
            color="#d62828",
            linewidth=1.0,
            label="드롭오프 위치",
            zorder=4,
        )
    if start_xy[0] is not None:
        ax.scatter(
            start_xy[0],
            start_xy[1],
            marker="o",
            s=90,
            color="#2a9d8f",
            edgecolor="white",
            linewidth=0.7,
            label="차량 현재 위치",
            zorder=4,
        )

    bound_points: List[Tuple[Optional[float], Optional[float]]] = [
        pickup_xy,
        dropoff_xy,
        start_xy,
    ]
    if prev_xs and prev_ys:
        bound_points.extend(list(zip(prev_xs, prev_ys)))
    if new_xs and new_ys:
        bound_points.extend(list(zip(new_xs, new_ys)))

    min_x, max_x, min_y, max_y = compute_bounds(bound_points)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", linewidth=0.4, alpha=0.4)
    ax.set_xlabel("경도", fontsize=8)
    ax.set_ylabel("위도", fontsize=8)
    ax.tick_params(labelsize=8)

    ax.set_title(
        f"요청 {event.request.passenger_id} → 차량 {event.vehicle_id} 배정\n"
        f"(비용: {event.cost:,.2f}, 픽업 노드: {event.request.pickup_node}, 드롭오프 노드: {event.request.dropoff_node})",
        fontsize=10,
    )

    text_y = max_y - (max_y - min_y) * 0.12
    ax.text(
        min_x + (max_x - min_x) * 0.02,
        text_y,
        f"이전 경로: {format_stop_sequence(event.previous_path)}",
        fontsize=8,
        color="#555555",
        bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7, edgecolor="none"),
    )
    ax.text(
        min_x + (max_x - min_x) * 0.02,
        text_y - (max_y - min_y) * 0.07,
        f"신규 경로: {format_stop_sequence(event.new_path)}",
        fontsize=8,
        color="#1b263b",
        bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7, edgecolor="none"),
    )

    deduplicate_legend(ax, fontsize=8)


def visualize_assignment_events(events: Sequence[AssignmentEvent], graph: nx.Graph, save_path: Optional[str] = None) -> None:
    if not events:
        print("시각화할 배차 이벤트가 없습니다.")
        return

    cols = 2 if len(events) > 1 else 1
    rows = (len(events) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(7.5 * cols, 6 * rows))
    axes_array = np.array(axes).reshape(rows, cols)
    axes_flat: List[plt.Axes] = list(axes_array.flatten())

    vehicle_color_map: Dict[int, str] = {}
    fallback_idx = 0
    for event in events:
        if event.vehicle_id in vehicle_color_map:
            continue
        if event.vehicle_id in PRESET_VEHICLE_COLORS:
            vehicle_color_map[event.vehicle_id] = PRESET_VEHICLE_COLORS[event.vehicle_id]
        else:
            fallback_color = FALLBACK_VEHICLE_COLORS[fallback_idx % len(FALLBACK_VEHICLE_COLORS)]
            vehicle_color_map[event.vehicle_id] = fallback_color
            fallback_idx += 1

    for idx, event in enumerate(events):
        ax = axes_flat[idx]
        color = vehicle_color_map.get(event.vehicle_id, FALLBACK_VEHICLE_COLORS[0])
        plot_assignment_event(ax, event, graph, color=color)

    for ax in axes_flat[len(events):]:
        ax.axis("off")

    if rows > 1 or cols > 1:
        fig.suptitle("DRT 배차 경로 비교 (배차 전/후)", fontsize=12)

    plt.tight_layout(rect=(0, 0, 1, 0.97))
    if save_path:
        plt.savefig(save_path, dpi=200)
        print(f"[저장 완료] {save_path}")
    else:
        plt.show()


# --------------------------------------------------------------------------------------
# 데모 실행 클래스
# --------------------------------------------------------------------------------------


class NetworkAssignmentDemo:
    """무작위 요청을 생성하고 배차 이벤트를 기록 및 시각화."""

    def __init__(self, num_vehicles: int = 2, vehicle_capacity: int = 4, seed: int = 42) -> None:
        self.graph = load_network_graph()
        self.engine = DRTAssignmentEngine(self.graph)
        self.random = random.Random(seed)
        self.vehicles: List[VehicleState] = []

        for idx in range(num_vehicles):
            start_node = select_random_node(self.graph)
            vehicle = VehicleState(
                vehicle_id=idx + 1,
                current_node=start_node,
                capacity=vehicle_capacity,
                onboard_passengers=0,
            )
            self.vehicles.append(vehicle)

    def _random_request(self, idx: int) -> PassengerRequest:
        pickup = select_random_node(self.graph)
        dropoff = select_random_node(self.graph)
        while dropoff == pickup:
            dropoff = select_random_node(self.graph)
        return PassengerRequest(
            passenger_id=f"demo_{idx:03d}",
            pickup_node=pickup,
            dropoff_node=dropoff,
        )

    def process_request(self, request: PassengerRequest) -> Optional[AssignmentEvent]:
        assigned_vehicle, new_path, cost = self.engine.assign_request(self.vehicles, request)
        if assigned_vehicle is None:
            print(f"[요청 {request.passenger_id}] 배차 실패")
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
        )

        assigned_vehicle.path = new_path

        print(f"[요청 {request.passenger_id}] 차량 {assigned_vehicle.vehicle_id} 배정 (비용: {cost:,.2f})")
        print(f"   - 픽업 노드: {request.pickup_node}, 드롭오프 노드: {request.dropoff_node}")
        print(f"   - 기존 경로: {format_stop_sequence(previous_path)}")
        print(f"   - 신규 경로: {format_stop_sequence(new_path)}\n")

        return event

    def run_demo(self, num_requests: int) -> List[AssignmentEvent]:
        events: List[AssignmentEvent] = []
        for idx in range(1, num_requests + 1):
            request = self._random_request(idx)
            event = self.process_request(request)
            if event:
                events.append(event)
        return events


# --------------------------------------------------------------------------------------
# 메인
# --------------------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="네트워크 기반 DRT 배차 경로 시각화 데모")
    parser.add_argument("--requests", type=int, default=10, help="무작위 요청 개수 (기본 5)")
    parser.add_argument("--save", type=str, default=None, help="PNG 저장 경로 (미지정 시 화면 표시)")
    parser.add_argument("--vehicles", type=int, default=2, help="데모용 차량 수 (기본 2)")
    parser.add_argument("--capacity", type=int, default=4, help="차량 용량 (기본 4)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    demo = NetworkAssignmentDemo(num_vehicles=args.vehicles, vehicle_capacity=args.capacity)
    events = demo.run_demo(args.requests)
    visualize_assignment_events(events, demo.graph, save_path=args.save)


if __name__ == "__main__":
    main()

