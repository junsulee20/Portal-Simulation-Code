#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
from datetime import datetime, timedelta
import networkx as nx
import pickle
from enum import Enum
from collections import defaultdict, deque
import time
import os
import re


class VehicleStatus(Enum):
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    TRAVELING_TO_PICKUP = "TRAVELING_TO_PICKUP"
    PICKING_UP = "PICKING_UP"
    TRAVELING_TO_DROPOFF = "TRAVELING_TO_DROPOFF"
    DROPPING_OFF = "DROPPING_OFF"
    RETURNING = "RETURNING"
    OFF_DUTY = "OFF_DUTY"


class PassengerStatus(Enum):
    REQUESTED = "REQUESTED"
    ASSIGNED = "ASSIGNED"
    PICKED_UP = "PICKED_UP"
    DROPPED_OFF = "DROPPED_OFF"
    CANCELLED = "CANCELLED"


class Location:
    def __init__(self, node_id):
        self.node_id = node_id


class Vehicle:
    def __init__(self, vehicle_id, vehicle_no, depot_location, depot_name, service_area="BOTH"):
        self.vehicle_id = vehicle_id
        self.vehicle_no = vehicle_no
        self.status = VehicleStatus.OFF_DUTY
        self.current_location = depot_location
        self.depot_location = depot_location
        self.depot_name = depot_name
        self.service_area = service_area  # "INSIDE_ONLY" 또는 "BOTH"
        self.assigned_passenger = None
        self.service_start_time = None
        self.service_end_time = None
        self.daily_services = 0
        self.accurate_schedule = {}
        self.work_start = None
        self.work_end = None
        self.actual_work_hours = 0
        self.total_distance = 0.0
        self.total_service_time = 0.0
        # 점심시간 조정 창 목록 [(start_dt, end_dt), ...]
        self.lunch_windows = []


class Passenger:
    def __init__(self, demand_id, customer_id, request_time, pickup_location, dropoff_location, mode, is_outside_area=False, pickup_depot_name=None, dropoff_depot_name=None):
        self.demand_id = demand_id
        self.customer_id = customer_id
        self.request_time = request_time
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location
        self.mode = mode
        self.is_outside_area = is_outside_area
        self.status = PassengerStatus.REQUESTED
        self.assigned_vehicle = None
        self.assigned_time = None
        self.pickup_time = None
        self.dropoff_time = None
        self.call_waiting_time = 0
        self.pickup_waiting_time = 0
        self.service_travel_time = 0
        self.total_trip_time = 0
        self.pickup_depot_name = pickup_depot_name
        self.dropoff_depot_name = dropoff_depot_name


class ScheduledIncreaseWithShiftSimulation:
    def __init__(self):
        self.vehicles = {}
        self.passengers = {}
        self.network_graph = None
        self.depot_info = {}
        self.pending_passengers = deque()
        self.service_records = []
        self.demand_call_log = []
        self.vehicle_service_log = defaultdict(list)
        self.path_cache = {}
        self.assigned_demands = set()
        self.simulation_start_time = None
        self.total_seconds = 0
        self.processed_seconds = 0
        self.hourly_speed_factors = {h: 0.33 for h in range(24)}
        self.base_speed_factor_assumed = 0.33
        self._routing_hour = 0
        # 추가 차량/진행 로그 추적
        self.added_vehicle_ids = set()
        self.progress_log = []
        # 관내/관외 및 권역 관련 실험 파라미터 (실행 시 주입)
        self.force_both_service_area = False
        self.region_strict_ratio = 0.0

    def _ensure_csv_path(self, path):
        try:
            p = str(path)
            if not p.lower().endswith('.csv'):
                return p + '.csv'
            return p
        except Exception:
            return path

    def _sanitize_filename(self, name):
        try:
            import re
            # Windows 금지문자 및 공백을 '_'로 치환
            sanitized = re.sub(r'[\\/:*?"<>| ]+', '_', str(name))
            # 연속 '_' 정리
            sanitized = re.sub(r"_+", "_", sanitized).strip('_')
            return sanitized
        except Exception:
            return name

    def _finalize_output_path(self, path):
        try:
            directory = os.path.dirname(path)
            base = os.path.basename(path)
            base = self._sanitize_filename(base)
            final_path = os.path.join(directory if directory else '', base)
            final_path = self._ensure_csv_path(final_path)
            return final_path
        except Exception:
            return self._ensure_csv_path(path)

    # --- 공통 로드 함수들 ---
    def load_network(self):
        print('네트워크 로드 중...')
        try:
            with open('network/main_network_graph.pkl', 'rb') as f:
                self.network_graph = pickle.load(f)
            print(f'   노드: {self.network_graph.number_of_nodes():,}개')
            print(f'   링크: {self.network_graph.number_of_edges():,}개')
            return True
        except Exception as e:
            print(f'   실패: {e}')
            return False

    def load_depot_info(self):
        print('차고지 정보 로드 중...')
        try:
            depot_df = pd.read_csv('network/depot_main_network_mapping_fixed.csv')
            for _, row in depot_df.iterrows():
                depot_name = row['region_name']
                self.depot_info[depot_name] = {
                    'node_id': row['nearest_node'],
                    'coordinates': (row['latitude'], row['longitude']),
                    'vehicles': row.get('vehicles', 10)
                }
            print(f'   차고지: {len(self.depot_info)}개')
            return True
        except Exception as e:
            print(f'   실패: {e}')
            return False

    def load_hourly_speed_factors(self, csv_path='data/hourly_speed_factors.csv'):
        try:
            if os.path.exists(csv_path):
                df = pd.read_csv(csv_path)
                loaded = 0
                for _, row in df.iterrows():
                    try:
                        hour = int(row['hour'])
                        factor = float(row['factor'])
                        if 0 <= hour <= 23 and factor > 0:
                            self.hourly_speed_factors[hour] = factor
                            loaded += 1
                    except Exception:
                        continue
                print(f"시간대별 속도 계수 로드: {loaded}개 (기본 계수 {self.base_speed_factor_assumed})")
            else:
                print(f"시간대별 속도 계수 파일 없음: {csv_path} (기본 {self.base_speed_factor_assumed} 적용)")
        except Exception as e:
            print(f"시간대별 속도 계수 로드 실패: {e} (기본값 유지)")

    def load_vehicles(self):
        print('기본 63대 차량 로드 중...')
        try:
            vehicle_mapping = pd.read_csv('network/fixed_vehicle_mapping_63.csv')
            depot_vehicle_count = defaultdict(int)
            for _, row in vehicle_mapping.iterrows():
                vehicle_id = int(row['vehicle_id'])
                vehicle_no = row['vehicle_no']
                depot_name = row['depot']
                if depot_name not in self.depot_info:
                    continue
                depot_location = Location(self.depot_info[depot_name]['node_id'])
                depot_vehicle_count[depot_name] += 1
                service_area = "INSIDE_ONLY" if (depot_vehicle_count[depot_name] % 2 == 1) else "BOTH"
                vehicle = Vehicle(
                    vehicle_id=vehicle_id,
                    vehicle_no=vehicle_no,
                    depot_location=depot_location,
                    depot_name=depot_name,
                    service_area=service_area
                )
                accurate_schedule = {hour: False for hour in range(24)}
                vehicle.accurate_schedule = accurate_schedule
                vehicle.work_start = "06:00:00"
                vehicle.work_end = "18:00:00"
                vehicle.actual_work_hours = 0.0
                self.vehicles[vehicle_id] = vehicle
            print(f'   차량: {len(self.vehicles)}대 로드')
            return True
        except Exception as e:
            print(f'   실패: {e}')
            print('   기본 차량 생성 중...')
            depot_names = list(self.depot_info.keys())
            for vehicle_id in range(1, 64):
                depot_name = depot_names[vehicle_id % len(depot_names)]
                depot_location = Location(self.depot_info[depot_name]['node_id'])
                service_area = "INSIDE_ONLY" if vehicle_id % 2 == 1 else "BOTH"
                vehicle = Vehicle(
                    vehicle_id=vehicle_id,
                    vehicle_no=f"차량{vehicle_id:02d}",
                    depot_location=depot_location,
                    depot_name=depot_name,
                    service_area=service_area
                )
                accurate_schedule = {hour: (6 <= hour <= 18) for hour in range(24)}
                vehicle.accurate_schedule = accurate_schedule
                vehicle.work_start = "06:00:00"
                vehicle.work_end = "18:00:00"
                vehicle.actual_work_hours = 12.0
                self.vehicles[vehicle_id] = vehicle
            print(f'   기본 차량: {len(self.vehicles)}대 생성')
            return True

    def apply_force_both_service_area(self):
        if not self.force_both_service_area:
            return
        changed = 0
        for v in self.vehicles.values():
            if getattr(v, 'service_area', None) != 'BOTH':
                v.service_area = 'BOTH'
                changed += 1
        if changed:
            print(f'   관내외 겸용 강제 적용: {changed}대 BOTH로 설정')

    # --- 일정 기반 추가 차량 로딩 ---
    def _normalize_weekday(self, w):
        mapping = {
            '0': 0, '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6,
            'mon': 0, 'monday': 0, '월': 0,
            'tue': 1, 'tuesday': 1, '화': 1,
            'wed': 2, 'wednesday': 2, '수': 2,
            'thu': 3, 'thursday': 3, '목': 3,
            'fri': 4, 'friday': 4, '금': 4,
            'sat': 5, 'saturday': 5, '토': 5,
            'sun': 6, 'sunday': 6, '일': 6,
            'weekday': -1, 'weekdays': -1, '주중': -1,
            'weekend': -2, '주말': -2
        }
        key = str(w).strip().lower()
        return mapping.get(key, None)

    def _parse_time_str(self, s):
        s = str(s).strip()
        if s.isdigit():
            hour = max(0, min(23, int(s)))
            return f"{hour:02d}:00:00", hour
        for fmt in ["%H:%M:%S", "%H:%M"]:
            try:
                dt = datetime.strptime(s, fmt)
                return dt.strftime("%H:%M:%S"), dt.hour
            except Exception:
                continue
        return "06:00:00", 6

    def load_additional_scheduled_vehicles(self, date_str, csv_path='data/additional_depot_vehicles_schedule_template_v1.csv'):
        print('일정 기반 추가 차량 로드 중...')
        if not os.path.exists(csv_path):
            print(f'   일정 템플릿 파일이 없습니다: {csv_path} (추가 차량 없음)')
            return True
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print(f'   템플릿 로드 실패: {e}')
            return False

        def col(*names):
            for name in names:
                if name in df.columns:
                    return name
            return None

        depot_col = col('depot', 'depot_name', 'region_name')
        weekday_col = col('weekday', 'day', 'day_of_week')
        start_col = col('start_time', 'start', 'from', 'start_hour')
        end_col = col('end_time', 'end', 'to', 'end_hour')
        count_col = col('num_vehicles', 'count', 'vehicles')
        area_col = col('service_area', 'area')

        required = [depot_col, weekday_col, start_col, end_col, count_col]
        if any(x is None for x in required):
            print('   템플릿 필수 컬럼이 누락되었습니다. (필수: depot, weekday, start_time, end_time, num_vehicles)')
            return False

        target_weekday = pd.to_datetime(date_str).weekday()
        max_vehicle_id = max(self.vehicles.keys()) if self.vehicles else 0
        added = 0

        for _, row in df.iterrows():
            depot_name = str(row[depot_col]).strip()
            if depot_name not in self.depot_info:
                continue

            weekday_spec = self._normalize_weekday(row[weekday_col])
            if weekday_spec is None:
                continue
            if weekday_spec >= 0 and weekday_spec != target_weekday:
                continue
            if weekday_spec == -1 and target_weekday >= 5:
                continue
            if weekday_spec == -2 and target_weekday < 5:
                continue

            start_str, start_hour = self._parse_time_str(row[start_col])
            end_str, end_hour = self._parse_time_str(row[end_col])
            try:
                num_vehicles = int(row[count_col])
                num_vehicles = max(0, num_vehicles)
            except Exception:
                num_vehicles = 0
            service_area = str(row[area_col]).strip().upper() if area_col else 'BOTH'
            if service_area not in ['BOTH', 'INSIDE_ONLY']:
                service_area = 'BOTH'

            if num_vehicles <= 0:
                continue

            depot_node = self.depot_info[depot_name]['node_id']
            for _ in range(num_vehicles):
                max_vehicle_id += 1
                vehicle = Vehicle(
                    vehicle_id=max_vehicle_id,
                    vehicle_no=f"추가일정차량_{max_vehicle_id}",
                    depot_location=Location(depot_node),
                    depot_name=depot_name,
                    service_area=service_area
                )

                accurate_schedule = {h: False for h in range(24)}
                if start_hour == end_hour:
                    pass
                elif start_hour < end_hour:
                    for h in range(start_hour, end_hour):
                        accurate_schedule[h] = True
                else:
                    for h in range(start_hour, 24):
                        accurate_schedule[h] = True
                    for h in range(0, end_hour):
                        accurate_schedule[h] = True

                vehicle.accurate_schedule = accurate_schedule
                vehicle.work_start = start_str
                vehicle.work_end = end_str
                duration_hours = (end_hour - start_hour) % 24
                vehicle.actual_work_hours = float(duration_hours)
                self.vehicles[max_vehicle_id] = vehicle
                # 추가 차량 ID 기록
                try:
                    self.added_vehicle_ids.add(max_vehicle_id)
                except Exception:
                    pass
                added += 1

        print(f'   추가 차량 생성: {added}대 (일정 일치 시)')
        return True

    def log_demand_call_result(self, passenger, result_type, vehicle_id=None, assignment_time=None):
        call_record = {
            'demand_id': passenger.demand_id,
            'call_time': passenger.request_time,
            'result_type': result_type,
            'assignment_time': assignment_time,
            'vehicle_id': vehicle_id,
            'pickup_time': None,
            'dropoff_time': None,
            'wait_minutes': None,
            'service_minutes': None
        }
        if assignment_time and passenger.request_time:
            wait_seconds = (assignment_time - passenger.request_time).total_seconds()
            call_record['wait_minutes'] = wait_seconds / 60.0
        self.demand_call_log.append(call_record)
        return len(self.demand_call_log) - 1

    def update_demand_log(self, demand_id, pickup_time=None, dropoff_time=None):
        for record in self.demand_call_log:
            if record['demand_id'] == demand_id:
                if pickup_time:
                    record['pickup_time'] = pickup_time
                if dropoff_time:
                    record['dropoff_time'] = dropoff_time
                if record['pickup_time'] and record['dropoff_time']:
                    service_seconds = (record['dropoff_time'] - record['pickup_time']).total_seconds()
                    record['service_minutes'] = service_seconds / 60.0
                break

    def log_vehicle_service(self, vehicle_id, action, passenger_id=None, current_time=None):
        service_record = {
            'time': current_time,
            'action': action,
            'passenger_id': passenger_id
        }
        self.vehicle_service_log[vehicle_id].append(service_record)

    # --- 점심시간 조정 관련 ---
    def _is_in_lunch_break(self, vehicle, current_time):
        if not hasattr(vehicle, 'lunch_windows') or not vehicle.lunch_windows:
            return False
        for start_dt, end_dt in vehicle.lunch_windows:
            if start_dt <= current_time < end_dt:
                return True
        return False

    # [Removed] group-based and hour-based lunch adjustments

    def _normalize_schedule_with_lunch(self):
        # 점심 창이 겹치는 모든 시간대는 근무 가능(True)로 표준화하여 OFF_DUTY 전이를 방지
        for v in self.vehicles.items():
            pass
        for vid, v in self.vehicles.items():
            try:
                if not getattr(v, 'lunch_windows', None):
                    continue
                for sdt, edt in v.lunch_windows:
                    sh = int(getattr(sdt, 'hour', 0))
                    eh = int(getattr(edt, 'hour', sh))
                    # 창이跨시간일 수 있어 시작/종료 시각의 시간대를 모두 활성화
                    for h in {sh, eh}:
                        if 0 <= h <= 23:
                            v.accurate_schedule[h] = True
            except Exception:
                continue

    def _parse_hhmm(self, s):
        s = str(s).strip()
        if ':' in s:
            hh, mm = s.split(':', 1)
            return int(hh), int(mm)
        return int(s), 0

    def _parse_lunch_realloc_arg(self, realloc_str):
        # 예: "12->11:30=0.3,12->13=0.2; 11->12=0.1"
        # 쉼표(,)로 분리, 공백 허용. 여러 원천 시간 정의 가능
        if realloc_str is None:
            return {}
        rules = {}
        parts = [p.strip() for p in str(realloc_str).replace(';', ',').split(',') if p.strip()]
        for part in parts:
            if '->' not in part or '=' not in part:
                continue
            left, right = part.split('->', 1)
            dest, ratio_s = right.split('=', 1)
            try:
                src_h, src_m = self._parse_hhmm(left)
                dst_h, dst_m = self._parse_hhmm(dest)
                if not (0 <= src_h <= 23 and 0 <= src_m <= 59 and 0 <= dst_h <= 23 and 0 <= dst_m <= 59):
                    continue
                ratio = max(0.0, min(1.0, float(ratio_s.strip())))
            except Exception:
                continue
            key = (src_h, src_m)
            rules.setdefault(key, []).append({'dst_h': dst_h, 'dst_m': dst_m, 'ratio': ratio})
        return rules

    def adjust_lunch_reallocation(self, date_str, realloc_rules, duration_minutes=60):
        # 기존 점심시간이 있는 운전원 중, 특정 시각(예: 12시) 그룹에서 일부 비율을 다른 시각으로 이동
        try:
            target_date = pd.to_datetime(date_str).date()
        except Exception:
            target_date = datetime.now().date()

        summary = {}
        for (src_h, src_m), targets in realloc_rules.items():
            # 1) 소스 그룹 후보 식별: '소스 시각에 시작하는' 점심창 보유 운전원만 대상
            candidates = []  # [{vid, idx}] idx는 이동 대상 점심창 인덱스
            for vid, v in self.vehicles.items():
                has_windows = bool(getattr(v, 'lunch_windows', []))
                found = False
                if has_windows:
                    for idx, (sdt, edt) in enumerate(v.lunch_windows):
                        # 포함 여부: 소스 시각 (src_h:src_m)이 해당 점심 구간 [sdt, edt) 안에 있으면 후보
                        try:
                            src_dt = datetime(target_date.year, target_date.month, target_date.day, int(src_h), int(src_m), 0)
                            if sdt <= src_dt < edt:
                                candidates.append({'vid': vid, 'idx': idx})
                                found = True
                                break
                        except Exception:
                            continue
                if not found:
                    # 점심창이 없으면, 스케줄 갭(주변 True, 소스 False)을 '소스 점심'으로 간주하여 기본 창을 생성
                    try:
                        sh = int(src_h) % 24
                        before_h = (sh - 1) % 24
                        after_h = (sh + 1) % 24
                        is_src_off = not bool(v.accurate_schedule.get(sh, False))
                        # 점심 추정 조건: 소스 시각은 False이고, 이전과 이후 모두 True일 때만 점심으로 간주
                        neighbors_on = bool(v.accurate_schedule.get(before_h, False)) and bool(v.accurate_schedule.get(after_h, False))
                        if is_src_off and neighbors_on:
                            start_dt = datetime(target_date.year, target_date.month, target_date.day, int(src_h), int(src_m), 0)
                            end_dt = start_dt + timedelta(minutes=int(duration_minutes))
                            if not hasattr(v, 'lunch_windows'):
                                v.lunch_windows = []
                            v.lunch_windows.append((start_dt, end_dt))
                            candidates.append({'vid': vid, 'idx': len(v.lunch_windows) - 1})
                    except Exception:
                        pass

            candidates = sorted(candidates, key=lambda x: x['vid'])

            # 2) 타깃별로 비율만큼 재배치 (중복 없이 순차 할당)
            remaining = [c for c in candidates]
            total_targets = len(targets)
            for i, t in enumerate(targets):
                dst_h = int(t['dst_h'])
                dst_m = int(t['dst_m'])
                remain_cnt = len(remaining)
                if remain_cnt <= 0:
                    break
                # 마지막 타깃은 잔여 전원 이동하여 반올림 잔여 제거
                if i == total_targets - 1:
                    take = remain_cnt
                else:
                    take = int(remain_cnt * float(t['ratio']))
                    # 최소 1명도 안 나오는 경우는 skip
                    if take <= 0:
                        continue
                chosen = remaining[:take]
                remaining = remaining[take:]
                # 선택된 운전원들의 해당 점심창을 타깃으로 이동(해당 창만 교체)
                for item in chosen:
                    vid = item['vid']
                    idx = item['idx']
                    v = self.vehicles.get(vid)
                    if not v:
                        continue
                    start_dt = datetime(target_date.year, target_date.month, target_date.day, dst_h, dst_m, 0)
                    end_dt = start_dt + timedelta(minutes=int(duration_minutes))
                    if 0 <= idx < len(v.lunch_windows):
                        v.lunch_windows[idx] = (start_dt, end_dt)
                    # 근무 가능 상태 보장: 원래 소스 시각과 타깃 시각 모두 True로 복원/보장
                    try:
                        # 소스 점심 창 전후 한 시간도 근무 가능으로 복원해 배정 제외 꼬임 방지
                        for h in [int(src_h) - 1, int(src_h), int(src_h) + 1, int(dst_h) - 1, int(dst_h), int(dst_h) + 1]:
                            if 0 <= h % 24 <= 23:
                                v.accurate_schedule[h % 24] = True
                    except Exception:
                        pass
                summary.setdefault((src_h, src_m), []).append({'dst_h': dst_h, 'dst_m': dst_m, 'count': len(chosen)})

        # 로그 출력 요약
        for (sh, sm), items in summary.items():
            for it in items:
                print(f"   {sh:02d}:{sm:02d} 점심에서 {it['dst_h']:02d}:{it['dst_m']:02d}로 조정: {it['count']}명")

        return summary

    def apply_previous_day_operations(self, date_str):
        target_date = pd.to_datetime(date_str).date()
        prev_date = target_date - timedelta(days=1)
        # 야간 운행(전날 21-23시)과 새벽(0-5시) 모두 활성인 차량 식별
        night_shift_vehicles = []
        for vehicle_id, vehicle in self.vehicles.items():
            evening_schedule_prev = any(vehicle.accurate_schedule.get(h, False) for h in [21, 22, 23])
            morning_schedule_current = any(vehicle.accurate_schedule.get(h, False) for h in [0, 1, 2, 3, 4, 5])
            if evening_schedule_prev and morning_schedule_current:
                night_shift_vehicles.append(vehicle_id)

        print(f'   {prev_date.strftime("%m월 %d일")}부터 연속 운행 차량 처리 중: {len(night_shift_vehicles)}대')
        continuous_operation_vehicles = 0
        for vehicle_id in night_shift_vehicles:
            vehicle = self.vehicles[vehicle_id]
            # 간단한 가상 서비스 부여로 새벽 서비스 중 상태 보장
            fake_start_time = datetime(prev_date.year, prev_date.month, prev_date.day, 23, 0, 0)
            fake_end_time = datetime(target_date.year, target_date.month, target_date.day, 1, 0, 0)
            vehicle.assigned_passenger = Passenger(
                demand_id=f"PREV_DAY_{prev_date.strftime('%d')}_{vehicle_id}_2300",
                pickup_location=vehicle.depot_location,
                dropoff_location=vehicle.depot_location,
                request_time=fake_start_time,
                customer_id=f"FAKE_CUSTOMER_{vehicle_id}",
                mode='특별교통수단'
            )
            vehicle.status = VehicleStatus.TRAVELING_TO_DROPOFF
            vehicle.service_end_time = fake_end_time
            continuous_operation_vehicles += 1
        if continuous_operation_vehicles > 0:
            print(f'   {prev_date.strftime("%d일")}부터 연속 운행 중인 차량: {continuous_operation_vehicles}대')
            print(f'   {target_date.strftime("%d일")} 새벽 서비스 가능 차량 확보 완료')

    def load_accurate_schedules(self, date_str="2025-06-23"):
        print('정확한 차량 스케줄 로드 중...')
        try:
            date_suffix = date_str.replace('-', '')[4:]
            special_schedule_path = f'network/special_transport_schedules_june_2025/accurate_individual_vehicle_schedule_{date_suffix}.csv'
            try:
                schedule_df = pd.read_csv(special_schedule_path)
                print(f'특별교통수단 스케줄 로드: {date_str} ({len(schedule_df)}건)')
            except FileNotFoundError as e:
                print(f'날짜별 스케줄 파일 없음, 기본 파일 사용: {e}')
                schedule_df = pd.read_csv('network/accurate_individual_vehicle_schedule.csv')
            updated_count = 0
            for _, row in schedule_df.iterrows():
                vehicle_id = int(row['vehicle_id'])
                hour = int(row['hour'])
                is_active = row['is_active']
                work_start = row['work_start']
                work_end = row['work_end']
                actual_work_hours = float(row['actual_work_hours'])
                if isinstance(is_active, str):
                    is_active = is_active.lower() == 'true'
                else:
                    is_active = bool(is_active)
                if vehicle_id in self.vehicles:
                    vehicle = self.vehicles[vehicle_id]
                    vehicle.accurate_schedule[hour] = is_active
                    if hour == 0:
                        vehicle.work_start = work_start
                        vehicle.work_end = work_end
                        vehicle.actual_work_hours = actual_work_hours
                    updated_count += 1
            # end-exclusive 보정
            for vehicle_id, vehicle in self.vehicles.items():
                try:
                    work_end_str = getattr(vehicle, 'work_end', None)
                    if isinstance(work_end_str, str) and len(work_end_str) >= 2:
                        end_hour = int(work_end_str.split(':')[0])
                        if 0 <= end_hour <= 23 and vehicle.accurate_schedule.get(end_hour, False):
                            vehicle.accurate_schedule[end_hour] = False
                except Exception:
                    pass

            # 전날 연속 운행 처리(스케줄 기반)
            self.apply_previous_day_operations(date_str)

            print(f'   {updated_count}대 차량 스케줄 업데이트 완료')
            return True
        except Exception as e:
            print(f'   정확한 스케줄 로드 실패: {e}')
            print('   기본 6-18시 스케줄 적용 중...')
            for vehicle in self.vehicles.values():
                for hour in range(24):
                    vehicle.accurate_schedule[hour] = (6 <= hour <= 18)
                vehicle.work_start = "06:00:00"
                vehicle.work_end = "18:00:00"
                vehicle.actual_work_hours = 12.0
            print(f'   기본 스케줄 적용 완료')
            return True

    # --- 근무시간 조정 (일반 규칙: XtoY) ---
    def _parse_shift_rule(self, rule_str):
        if rule_str is None:
            return None
        s = str(rule_str).strip().lower()
        m = re.match(r'^(\d{1,2})\s*(?:to|->|➡|=>)\s*(\d{1,2})$', s)
        if not m:
            # 지원 포맷: "8to5", "8->5" 등. 그 외는 실패
            return None
        src = int(m.group(1))
        dst = int(m.group(2))
        if not (0 <= src <= 23 and 0 <= dst <= 23):
            return None
        return (src, dst)

    def _hour_str(self, hour):
        hour = int(hour) % 24
        return f"{hour:02d}:00:00"

    def _shift_schedule_map(self, schedule_map, delta):
        # 시간대 스케줄을 delta(시간)만큼 회전(이동)
        new_map = {h: False for h in range(24)}
        for h in range(24):
            if schedule_map.get(h, False):
                nh = (h + delta) % 24
                new_map[nh] = True
        return new_map

    def adjust_driver_shifts(self, rule_str, ratio=0.1):
        rule = self._parse_shift_rule(rule_str)
        if not rule:
            print(f"근무시간 조정 규칙 파싱 실패: '{rule_str}' (예: 6to4, 8to5)")
            return False
        src_hour, dst_hour = rule
        delta = (dst_hour - src_hour)  # 양수: 늦춤, 음수: 당김
        ratio = max(0.0, min(1.0, float(ratio)))
        ratio_pct = int(ratio * 100)
        print(f"근무시간 조정 적용: {src_hour}→{dst_hour} (비율 {ratio_pct}%)")

        # 대상 차량: work_start 시각이 src_hour 인 차량
        candidates = []
        for vid, v in self.vehicles.items():
            try:
                ws = getattr(v, 'work_start', None)
                start_h = int(str(ws).split(':')[0]) if isinstance(ws, str) and ':' in ws else None
                if start_h is not None and start_h == src_hour:
                    candidates.append(vid)
            except Exception:
                continue

        # 우선순위 없이 단순 오름차순 선택
        ordered = sorted(candidates)
        adjust_count = int(len(ordered) * ratio)
        to_adjust = ordered[:adjust_count]

        print(f"   대상 운전원: {len(candidates)}명 중 {len(to_adjust)}명 조정 (ID: {to_adjust})")

        for vid in to_adjust:
            v = self.vehicles.get(vid)
            if not v:
                continue
            # 기존 시각
            try:
                start_h = int(str(v.work_start).split(':')[0])
            except Exception:
                start_h = src_hour
            try:
                end_h = int(str(v.work_end).split(':')[0])
            except Exception:
                end_h = (start_h + int(v.actual_work_hours if getattr(v, 'actual_work_hours', None) else 12)) % 24

            # 새 시각: 시작은 dst_hour로 강제, 종료는 동일 시간만큼 이동
            new_start_h = dst_hour % 24
            new_end_h = (end_h + delta) % 24

            # 스케줄 맵 이동(회전)으로 중간 OFF 패턴 보존
            original_map = dict(v.accurate_schedule)
            shifted_map = self._shift_schedule_map(original_map, delta)

            # end-exclusive 보정
            if shifted_map.get(new_end_h, False):
                shifted_map[new_end_h] = False

            v.accurate_schedule = shifted_map
            v.work_start = self._hour_str(new_start_h)
            v.work_end = self._hour_str(new_end_h)
            # 실제 근무시간 추정 유지 (변경 전 값 보존)
            # v.actual_work_hours 그대로 둠
            print(f"   차량 {vid}: {start_h:02d}-{end_h:02d} → {new_start_h:02d}-{new_end_h:02d}")

        return True

    def load_daily_demands(self, date_str):
        print(f'{date_str} 특별교통수단 수요 로드 중...')
        try:
            demand_df = pd.read_csv('data/demand_main_network_mapped.csv')
            demand_df['receipt_time'] = pd.to_datetime(demand_df['receipt_time'])
            daily_special = demand_df[(demand_df['receipt_time'].dt.date == pd.to_datetime(date_str).date()) & (demand_df['mode'] == '특별교통수단')].copy()
            daily_special = daily_special.sort_values('receipt_time')
            print(f'   원본 {date_str} 특별교통수단: {len(daily_special)}건')
            outside_area_count = 0
            for i, (_, row) in enumerate(daily_special.iterrows()):
                customer_id = row['customer_id']
                unique_demand_id = f"{customer_id}_june23_{i+1:03d}"
                origin_area = str(row.get('origin1', ''))
                destination_area = str(row.get('destination1', ''))
                is_outside_area = (origin_area != '경기도 화성시') or (destination_area != '경기도 화성시')
                if is_outside_area:
                    outside_area_count += 1
                passenger = Passenger(
                    demand_id=unique_demand_id,
                    customer_id=customer_id,
                    request_time=row['receipt_time'],
                    pickup_location=Location(row['nearest_boarding_node']),
                    dropoff_location=Location(row['nearest_arrival_node']),
                    mode=row['mode'],
                    is_outside_area=is_outside_area,
                    pickup_depot_name=row.get('pickup_depot_name', None),
                    dropoff_depot_name=row.get('dropoff_depot_name', None)
                )
                self.passengers[unique_demand_id] = passenger
            print(f'   로드된 승객: {len(self.passengers)}명')
            print(f'   관외 지역 포함 여행: {outside_area_count}건')
            print(f'   관내 전용 여행: {len(self.passengers) - outside_area_count}건')
            return True
        except Exception as e:
            print(f'   실패: {e}')
            print('   샘플 수요 생성 중...')
            sample_times = ["07:30:00", "08:15:00", "09:00:00", "10:30:00", "11:45:00", "13:20:00", "14:10:00", "15:35:00", "16:50:00", "17:25:00"]
            depot_nodes = [info['node_id'] for info in self.depot_info.values()]
            for i, time_str in enumerate(sample_times):
                demand_id = f"SAMPLE_{i+1:03d}_june23"
                request_time = datetime.strptime(f'{date_str} {time_str}', '%Y-%m-%d %H:%M:%S')
                pickup_node = depot_nodes[i % len(depot_nodes)]
                dropoff_node = depot_nodes[(i + 1) % len(depot_nodes)]
                passenger = Passenger(
                    demand_id=demand_id,
                    customer_id=f"CUSTOMER_{i+1:03d}",
                    request_time=request_time,
                    pickup_location=Location(pickup_node),
                    dropoff_location=Location(dropoff_node),
                    mode='특별교통수단'
                )
                self.passengers[demand_id] = passenger
            print(f'   샘플 승객: {len(self.passengers)}명 생성')
            return True

    def get_shortest_path_time(self, from_node, to_node, current_time=None):
        cache_key = (from_node, to_node)
        try:
            if current_time is not None:
                hour = int(getattr(current_time, 'hour'))
            else:
                hour = int(self._routing_hour)
        except Exception:
            hour = 0
        if cache_key in self.path_cache:
            base_seconds = self.path_cache[cache_key]
        else:
            try:
                travel_time_minutes = nx.shortest_path_length(self.network_graph, from_node, to_node, weight='weight')
                base_seconds = travel_time_minutes * 60
                self.path_cache[cache_key] = base_seconds
            except Exception:
                base_seconds = 30 * 60
                self.path_cache[cache_key] = base_seconds
        factor = self.hourly_speed_factors.get(hour, self.base_speed_factor_assumed)
        if factor <= 0:
            factor = self.base_speed_factor_assumed
        scaled_seconds = base_seconds * (self.base_speed_factor_assumed / factor)
        return scaled_seconds

    # --- 상태 업데이트 및 배정 로직 ---
    def update_vehicle_status(self, current_time):
        current_hour = current_time.hour
        for vehicle in self.vehicles.values():
            is_work_time = vehicle.accurate_schedule.get(current_hour, False)
            # 새벽 시간 특례는 accurate_schedule을 그대로 신뢰
            if not is_work_time:
                if vehicle.status not in [VehicleStatus.OFF_DUTY]:
                    if vehicle.status in [VehicleStatus.TRAVELING_TO_PICKUP, VehicleStatus.PICKING_UP, VehicleStatus.TRAVELING_TO_DROPOFF, VehicleStatus.DROPPING_OFF]:
                        continue
                    else:
                        vehicle.status = VehicleStatus.OFF_DUTY
                        vehicle.current_location = vehicle.depot_location
            else:
                if vehicle.status == VehicleStatus.OFF_DUTY:
                    vehicle.status = VehicleStatus.IDLE

        # ASSIGNED → TRAVELING_TO_PICKUP 초기 전이 처리
        for vehicle in self.vehicles.values():
            if vehicle.assigned_passenger is None:
                continue
            if vehicle.status == VehicleStatus.ASSIGNED:
                passenger = vehicle.assigned_passenger
                try:
                    pickup_travel_time = self.get_shortest_path_time(
                        vehicle.current_location.node_id,
                        passenger.pickup_location.node_id,
                        current_time=current_time
                    )
                except Exception:
                    pickup_travel_time = 5 * 60
                vehicle.service_end_time = current_time + timedelta(seconds=pickup_travel_time)
                vehicle.status = VehicleStatus.TRAVELING_TO_PICKUP

        for vehicle in self.vehicles.values():
            if vehicle.service_end_time and current_time >= vehicle.service_end_time:
                if vehicle.status == VehicleStatus.DROPPING_OFF:
                    passenger = vehicle.assigned_passenger
                    if passenger:
                        passenger.dropoff_time = current_time
                        passenger.status = PassengerStatus.DROPPED_OFF
                        try:
                            passenger.service_travel_time = (passenger.dropoff_time - passenger.pickup_time).total_seconds() / 60
                        except Exception:
                            passenger.service_travel_time = 0
                        try:
                            passenger.total_trip_time = (passenger.dropoff_time - passenger.request_time).total_seconds() / 60
                        except Exception:
                            passenger.total_trip_time = 0
                        self.update_demand_log(passenger.demand_id, dropoff_time=current_time)
                        self.log_vehicle_service(vehicle.vehicle_id, 'DROPOFF', passenger.demand_id, current_time)
                        self.service_records.append({
                            'demand_id': passenger.demand_id,
                            'vehicle_id': vehicle.vehicle_id,
                            'vehicle_no': vehicle.vehicle_no,
                            'depot': vehicle.depot_name,
                            'request_time': passenger.request_time.strftime('%H:%M:%S') if passenger.request_time else '',
                            'assigned_time': passenger.assigned_time.strftime('%H:%M:%S') if passenger.assigned_time else '',
                            'pickup_time': passenger.pickup_time.strftime('%H:%M:%S') if passenger.pickup_time else '',
                            'dropoff_time': passenger.dropoff_time.strftime('%H:%M:%S') if passenger.dropoff_time else '',
                            'call_waiting_time': passenger.call_waiting_time,
                            'pickup_waiting_time': passenger.pickup_waiting_time,
                            'service_travel_time': passenger.service_travel_time,
                            'total_trip_time': passenger.total_trip_time,
                            'work_start': vehicle.work_start,
                            'work_end': vehicle.work_end,
                            'actual_work_hours': vehicle.actual_work_hours
                        })
                    vehicle.assigned_passenger = None
                    vehicle.service_start_time = None
                    vehicle.service_end_time = None
                    vehicle.daily_services += 1
                    current_hour = current_time.hour
                    is_work_time = vehicle.accurate_schedule.get(current_hour, False)
                    if is_work_time:
                        vehicle.status = VehicleStatus.IDLE
                        vehicle.current_location = vehicle.depot_location
                        self.log_vehicle_service(vehicle.vehicle_id, 'RETURN_IDLE', None, current_time)
                    else:
                        vehicle.status = VehicleStatus.OFF_DUTY
                elif vehicle.status == VehicleStatus.TRAVELING_TO_PICKUP:
                    passenger = vehicle.assigned_passenger
                    if passenger:
                        passenger.status = PassengerStatus.PICKED_UP
                        passenger.pickup_time = current_time
                        try:
                            passenger.pickup_waiting_time = (passenger.pickup_time - passenger.assigned_time).total_seconds() / 60
                        except Exception:
                            passenger.pickup_waiting_time = 0
                        self.update_demand_log(passenger.demand_id, pickup_time=current_time)
                        self.log_vehicle_service(vehicle.vehicle_id, 'PICKUP', passenger.demand_id, current_time)
                        boarding_seconds = 3 * 60
                        vehicle.service_end_time = current_time + timedelta(seconds=boarding_seconds)
                        vehicle.status = VehicleStatus.PICKING_UP

                elif vehicle.status == VehicleStatus.PICKING_UP:
                    passenger = vehicle.assigned_passenger
                    if passenger and current_time >= vehicle.service_end_time:
                        service_travel_seconds = self.get_shortest_path_time(
                            passenger.pickup_location.node_id,
                            passenger.dropoff_location.node_id,
                            current_time=current_time
                        )
                        vehicle.service_end_time = current_time + timedelta(seconds=service_travel_seconds)
                        vehicle.status = VehicleStatus.TRAVELING_TO_DROPOFF
                        vehicle.current_location = passenger.dropoff_location
                elif vehicle.status == VehicleStatus.TRAVELING_TO_DROPOFF:
                    vehicle.status = VehicleStatus.DROPPING_OFF
                    vehicle.service_end_time = current_time + timedelta(seconds=2 * 60)

    def assign_passenger_to_vehicle(self, passenger, current_time):
        if passenger.demand_id in self.assigned_demands:
            return False
        current_hour = current_time.hour
        available_vehicles = []
        for v in self.vehicles.values():
            if v.status == VehicleStatus.IDLE and v.assigned_passenger is None:
                is_working = v.accurate_schedule.get(current_hour, False)
                # 점심시간 조정 창에서는 배정 제외 (단, 이미 ASSIGNED/운행 중이면 제외하지 않음)
                if is_working and self._is_in_lunch_break(v, current_time):
                    if v.assigned_passenger is None and v.status == VehicleStatus.IDLE:
                        is_working = False
                if passenger.is_outside_area and v.service_area == "INSIDE_ONLY":
                    continue
                if is_working and hasattr(v, 'work_end'):
                    try:
                        work_end_hour = int(v.work_end.split(':')[0])
                        if current_hour >= work_end_hour - 1 and current_hour < work_end_hour:
                            is_working = False
                    except Exception:
                        pass
                if is_working:
                    next_hour = (current_hour + 1) % 24
                    next_hour_active = v.accurate_schedule.get(next_hour, False) if hasattr(v, "accurate_schedule") else False
                    if not next_hour_active:
                        try:
                            pickup_travel_time = self.get_shortest_path_time(v.current_location.node_id, passenger.pickup_location.node_id, current_time=current_time) / 60
                            service_travel_time = self.get_shortest_path_time(passenger.pickup_location.node_id, passenger.dropoff_location.node_id, current_time=current_time) / 60
                            total_service_minutes = pickup_travel_time + 3 + service_travel_time + 3
                            service_completion_time = current_time + timedelta(minutes=total_service_minutes)
                            next_hour_start = current_time.replace(minute=0, second=0) + timedelta(hours=1)
                            if service_completion_time >= next_hour_start:
                                is_working = False
                        except Exception:
                            is_working = False
                if is_working:
                    available_vehicles.append(v)
        if not available_vehicles:
            return False
        # 권역 우선 비율 적용: 픽업 권역 명이 있고 비율 조건이면 동일 권역 차량만 후보로 제한
        candidate_vehicles = available_vehicles
        try:
            if self.region_strict_ratio > 0:
                import random
                prefer_same = random.random() < float(self.region_strict_ratio)
                pickup_region = getattr(passenger, 'pickup_depot_name', None)
                if prefer_same and pickup_region:
                    same_region = [v for v in available_vehicles if getattr(v, 'depot_name', None) == pickup_region]
                    if same_region:
                        candidate_vehicles = same_region
        except Exception:
            candidate_vehicles = available_vehicles
        best_vehicle = None
        best_time = float('inf')
        for vehicle in candidate_vehicles:
            travel_seconds = self.get_shortest_path_time(vehicle.current_location.node_id, passenger.pickup_location.node_id, current_time=current_time)
            if travel_seconds < best_time:
                best_time = travel_seconds
                best_vehicle = vehicle
        if best_vehicle:
            self.assigned_demands.add(passenger.demand_id)
            passenger.assigned_vehicle = best_vehicle
            passenger.assigned_time = current_time
            passenger.status = PassengerStatus.ASSIGNED
            passenger.call_waiting_time = (passenger.assigned_time - passenger.request_time).total_seconds() / 60
            best_vehicle.assigned_passenger = passenger
            best_vehicle.status = VehicleStatus.TRAVELING_TO_PICKUP
            best_vehicle.service_start_time = current_time
            best_vehicle.service_end_time = current_time + timedelta(seconds=best_time)
            self.log_demand_call_result(passenger, 'ASSIGNED', best_vehicle.vehicle_id, current_time)
            self.log_vehicle_service(best_vehicle.vehicle_id, 'ASSIGNED', passenger.demand_id, current_time)
            return True
        return False

    def process_pending_passengers(self, current_time):
        if not self.pending_passengers:
            return
        current_hour = current_time.hour
        available_vehicles = []
        for vehicle in self.vehicles.values():
            if vehicle.status == VehicleStatus.IDLE and vehicle.assigned_passenger is None:
                is_working = vehicle.accurate_schedule.get(current_hour, False)
                # 점심시간 조정 창에서는 배정 제외 (단, 이미 ASSIGNED/운행 중이면 제외하지 않음)
                if is_working and self._is_in_lunch_break(vehicle, current_time):
                    if vehicle.assigned_passenger is None and vehicle.status == VehicleStatus.IDLE:
                        is_working = False
                if is_working and hasattr(vehicle, 'work_end'):
                    try:
                        work_end_hour = int(vehicle.work_end.split(':')[0])
                        if current_hour >= work_end_hour - 1 and current_hour < work_end_hour:
                            is_working = False
                    except Exception:
                        pass
                if is_working:
                    next_hour = (current_hour + 1) % 24
                    next_hour_active = vehicle.accurate_schedule.get(next_hour, False) if hasattr(vehicle, "accurate_schedule") else False
                    if not next_hour_active:
                        try:
                            # pending assignment conservative check: skip complex ETA calc if unsafe
                            is_working = False
                        except Exception:
                            is_working = False
                if is_working:
                    available_vehicles.append(vehicle)

        pending_passengers_list = [p for p in self.pending_passengers if p.demand_id not in self.assigned_demands]
        outside_passengers = [p for p in pending_passengers_list if p.is_outside_area]
        inside_passengers = [p for p in pending_passengers_list if not p.is_outside_area]
        both_vehicles = [v for v in available_vehicles if v.service_area == "BOTH"]
        inside_only_vehicles = [v for v in available_vehicles if v.service_area == "INSIDE_ONLY"]
        outside_assignments = min(len(outside_passengers), len(both_vehicles))
        remaining_both = len(both_vehicles) - outside_assignments
        inside_assignments = min(len(inside_passengers), remaining_both + len(inside_only_vehicles))
        total_assignments = outside_assignments + inside_assignments
        if total_assignments > 0:
            print(f"{current_time.strftime('%H:%M')} 즉시배정: 관외{len(outside_passengers)}명+관내{len(inside_passengers)}명, "
                  f"겸용{len(both_vehicles)}대+전용{len(inside_only_vehicles)}대 → 관외{outside_assignments}+관내{inside_assignments}건 배정")
        passengers_to_remove = []
        assignments = []
        for i in range(outside_assignments):
            assignments.append((outside_passengers[i], both_vehicles[i]))
        remaining_vehicles = both_vehicles[outside_assignments:] + inside_only_vehicles
        for i in range(inside_assignments):
            assignments.append((inside_passengers[i], remaining_vehicles[i]))
        for passenger, vehicle in assignments:
            vehicle.status = VehicleStatus.ASSIGNED
            vehicle.assigned_passenger = passenger
            passenger.status = PassengerStatus.ASSIGNED
            passenger.assigned_vehicle = vehicle
            passenger.assigned_time = current_time
            try:
                passenger.call_waiting_time = (passenger.assigned_time - passenger.request_time).total_seconds() / 60
            except Exception:
                passenger.call_waiting_time = 0
            pickup_time = 5 * 60
            vehicle.service_end_time = current_time + timedelta(seconds=pickup_time)
            service_time = 25 * 60
            vehicle.service_end_time = current_time + timedelta(seconds=pickup_time + service_time)
            self.assigned_demands.add(passenger.demand_id)
            self.log_demand_call_result(passenger, 'ASSIGNED', vehicle.vehicle_id, current_time)
            self.log_vehicle_service(vehicle.vehicle_id, 'ASSIGNED', passenger.demand_id, current_time)
            passengers_to_remove.append(passenger)
        for passenger in passengers_to_remove:
            self.pending_passengers.remove(passenger)

    def process_second(self, current_time):
        self.update_vehicle_status(current_time)
        self.process_pending_passengers(current_time)
        for passenger in self.passengers.values():
            if (passenger.status == PassengerStatus.REQUESTED and passenger.request_time <= current_time and passenger.demand_id not in self.assigned_demands):
                if self.assign_passenger_to_vehicle(passenger, current_time):
                    continue
                else:
                    if passenger not in self.pending_passengers:
                        self.pending_passengers.append(passenger)
                        self.log_demand_call_result(passenger, 'WAITING', None, current_time)
        self.process_pending_passengers(current_time)

    def run_simulation(self, date_str='2025-06-23'):
        print(f'\n{date_str} 24시간 초단위 시뮬레이션 시작')
        print('초 단위 정밀 시뮬레이션')
        print('중복 배정 완전 제거')
        print('실시간 진행 상황 모니터링')
        print('=' * 80)
        self.date_str = date_str
        self.simulation_start_time = time.time()
        start_time = datetime.strptime(f'{date_str} 00:00:00', '%Y-%m-%d %H:%M:%S')
        end_time = datetime.strptime(f'{date_str} 23:59:59', '%Y-%m-%d %H:%M:%S')
        current_time = start_time
        self.total_seconds = int((end_time - start_time).total_seconds()) + 1
        print(f'총 시뮬레이션 시간: {self.total_seconds:,}초 (24시간)')
        progress_interval = 300
        last_progress = 0
        while current_time <= end_time:
            self.process_second(current_time)
            self.processed_seconds += 1
            seconds_elapsed = (current_time - start_time).total_seconds()
            if seconds_elapsed - last_progress >= progress_interval:
                current_hour = current_time.hour
                active_vehicles = 0
                lunch_blocked_ids = []
                for v in self.vehicles.values():
                    is_active = v.accurate_schedule.get(current_hour, False)
                    # 점심시간 조정 창 동안(미배정·IDLE) 활성에서 제외. 운행 중이면 포함
                    if is_active and self._is_in_lunch_break(v, current_time):
                        if v.assigned_passenger is None and v.status == VehicleStatus.IDLE:
                            lunch_blocked_ids.append(v.vehicle_id)
                            is_active = False
                    if is_active:
                        active_vehicles += 1
                available_vehicles = 0
                busy_vehicles = 0
                lunch_blocked_available_ids = []
                for v in self.vehicles.values():
                    is_active = v.accurate_schedule.get(current_hour, False)
                    # 점심시간 조정 창 동안(미배정·IDLE) 가용에서 제외
                    if is_active and self._is_in_lunch_break(v, current_time):
                        if v.assigned_passenger is None and v.status == VehicleStatus.IDLE:
                            lunch_blocked_available_ids.append(v.vehicle_id)
                            is_active = False
                    if is_active and hasattr(v, 'work_end') and v.status == VehicleStatus.IDLE:
                        try:
                            work_end_hour = int(v.work_end.split(':')[0])
                            if current_hour >= work_end_hour - 1 and current_hour < work_end_hour:
                                is_active = False
                        except Exception:
                            pass
                    if is_active:
                        if v.status == VehicleStatus.IDLE:
                            available_vehicles += 1
                        elif v.status not in [VehicleStatus.OFF_DUTY]:
                            busy_vehicles += 1
                unassigned_waiting = len(self.pending_passengers)
                assigned_waiting = sum(1 for p in self.passengers.values() if p.status == PassengerStatus.ASSIGNED)
                total_waiting = unassigned_waiting + assigned_waiting
                completed_services = len(self.service_records)
                assigned_count = len(self.assigned_demands)
                progress_percent = (self.processed_seconds / self.total_seconds) * 100
                elapsed_real_time = time.time() - self.simulation_start_time
                base_msg = (f"{current_time.strftime('%H:%M')} ({progress_percent:.1f}%) - "
                            f"가동:{active_vehicles}대, 운행가능:{available_vehicles}대, 서비스중:{busy_vehicles}대, "
                            f"대기:{total_waiting}명(미배정:{unassigned_waiting}, 차량대기:{assigned_waiting}), "
                            f"배정:{assigned_count}건, 완료:{completed_services}건 [실제경과: {elapsed_real_time:.1f}초]")
                # 점심 영향 카운트(항상 표시): 점심(IDLE·미배정) 제외 수, 점심 중 운행 중 수
                try:
                    lunch_in_service = 0
                    lunch_total = 0
                    for v in self.vehicles.values():
                        if self._is_in_lunch_break(v, current_time):
                            lunch_total += 1
                            if v.assigned_passenger is not None or v.status != VehicleStatus.IDLE:
                                lunch_in_service += 1
                    base_msg += f" | 점심(총/IDLE제외/운행중): {lunch_total}/{len(lunch_blocked_ids)}/{lunch_in_service}"
                except Exception:
                    pass
                # 추가 차량 현황 표시: 현재 활성 시간대에 속하는 추가 차량 수
                try:
                    added_active = 0
                    added_total = len(getattr(self, 'added_vehicle_ids', []))
                    for vid in getattr(self, 'added_vehicle_ids', []):
                        v = self.vehicles.get(vid)
                        if not v:
                            continue
                        if v.accurate_schedule.get(current_hour, False):
                            added_active += 1
                    base_msg += f" | 추가차량(총/가동): {added_total}/{added_active}"
                except Exception:
                    pass
                if getattr(self, 'debug_lunch', False):
                    # 샘플 일부만 출력
                    sample_n = max(0, int(getattr(self, 'debug_lunch_sample', 5)))
                    lunch_sample = lunch_blocked_ids[:sample_n]
                    avail_sample = lunch_blocked_available_ids[:sample_n]
                    base_msg += (f" [활성제외샘플:{lunch_sample} 가용제외샘플:{avail_sample}]")
                print(base_msg)
                # 진행 로그를 CSV용 메모리에 적재
                try:
                    self.progress_log.append({
                        'time': current_time.strftime('%H:%M:%S'),
                        'active': active_vehicles,
                        'available': available_vehicles,
                        'busy': busy_vehicles,
                        'waiting_total': total_waiting,
                        'waiting_unassigned': unassigned_waiting,
                        'waiting_assigned': assigned_waiting,
                        'assigned_count': assigned_count,
                        'completed': completed_services,
                        'lunch_total': lunch_total if 'lunch_total' in locals() else 0,
                        'lunch_blocked_idle': len(lunch_blocked_ids),
                        'lunch_in_service': lunch_in_service if 'lunch_in_service' in locals() else 0,
                        'added_total': added_total if 'added_total' in locals() else len(getattr(self, 'added_vehicle_ids', [])),
                        'added_active': added_active if 'added_active' in locals() else 0
                    })
                except Exception:
                    pass
                last_progress = seconds_elapsed
            current_time += timedelta(seconds=1)
        total_real_time = time.time() - self.simulation_start_time
        print(f'\n초단위 24시간 시뮬레이션 완료!')
        print(f'최종 결과:')
        print(f'   시뮬레이션 시간: {self.total_seconds:,}초 (24시간)')
        print(f'   실제 소요 시간: {total_real_time:.1f}초')
        print(f'   시간 압축비: {self.total_seconds/total_real_time:.1f}배 고속 처리')
        print(f'   총 수요: {len(self.passengers)}건')
        print(f'   배정 완료: {len(self.assigned_demands)}건')
        print(f'   서비스 완료: {len(self.service_records)}건')
        print(f'   대기 중: {len(self.pending_passengers)}명')
        print(f'   중복 배정: 0건 (완전 제거)')
        return True

    # --- 결과 저장/로그: 메인 결과에 통합 ---
    def save_results(self, output_file='results/scheduled_increase_with_shift_20250623.csv'):
        print(f'\n초단위 시뮬레이션 결과 저장 중...')
        if not os.path.exists('results'):
            os.makedirs('results')
        if self.service_records:
            def _parse_time_to_dt(t_str):
                try:
                    if t_str is None or t_str == '':
                        return None
                    return datetime.strptime(f"2000-01-01 {t_str}", '%Y-%m-%d %H:%M:%S')
                except Exception:
                    return None

            enriched_records = []
            for rec in self.service_records:
                demand_id = rec.get('demand_id')
                p = self.passengers.get(demand_id)
                pickup_node_id = None
                dropoff_node_id = None
                pickup_x = None
                pickup_y = None
                dropoff_x = None
                dropoff_y = None
                pickup_depot_name = None
                dropoff_depot_name = None
                if p is not None:
                    pickup_node_id = getattr(getattr(p, 'pickup_location', None), 'node_id', None)
                    dropoff_node_id = getattr(getattr(p, 'dropoff_location', None), 'node_id', None)
                    if self.network_graph and pickup_node_id in self.network_graph.nodes:
                        n = self.network_graph.nodes[pickup_node_id]
                        pickup_x = n.get('longitude', None)
                        pickup_y = n.get('latitude', None)
                    if self.network_graph and dropoff_node_id in self.network_graph.nodes:
                        n = self.network_graph.nodes[dropoff_node_id]
                        dropoff_x = n.get('longitude', None)
                        dropoff_y = n.get('latitude', None)
                    pickup_depot_name = getattr(p, 'pickup_depot_name', None)
                    dropoff_depot_name = getattr(p, 'dropoff_depot_name', None)
                new_rec = dict(rec)
                assigned_time_str = new_rec.get('assigned_time')
                request_time_str = new_rec.get('request_time')
                pickup_time_str = new_rec.get('pickup_time')
                assigned_dt = _parse_time_to_dt(assigned_time_str)
                request_dt = _parse_time_to_dt(request_time_str)
                pickup_dt = _parse_time_to_dt(pickup_time_str)
                call_waiting_minutes = new_rec.get('call_waiting_time')
                pickup_waiting_minutes = new_rec.get('pickup_waiting_time')
                if call_waiting_minutes in [None, '']:
                    if assigned_dt and request_dt:
                        call_waiting_minutes = (assigned_dt - request_dt).total_seconds() / 60
                if pickup_waiting_minutes in [None, '']:
                    if pickup_dt and assigned_dt:
                        pickup_waiting_minutes = (pickup_dt - assigned_dt).total_seconds() / 60
                new_rec.update({
                    'pickup_node_id': pickup_node_id,
                    'dropoff_node_id': dropoff_node_id,
                    'pickup_x': pickup_x,
                    'pickup_y': pickup_y,
                    'dropoff_x': dropoff_x,
                    'dropoff_y': dropoff_y,
                    'pickup_depot_name': pickup_depot_name,
                    'dropoff_depot_name': dropoff_depot_name,
                    'call_waiting_time': call_waiting_minutes,
                    'pickup_waiting_time': pickup_waiting_minutes,
                })
                enriched_records.append(new_rec)
            results_df = pd.DataFrame(enriched_records)
            cols = list(results_df.columns)
            if 'dropoff_time' in cols:
                for c in ['call_waiting_time', 'pickup_waiting_time']:
                    if c in cols:
                        cols.remove(c)
                idx = cols.index('dropoff_time')
                cols = cols[:idx+1] + ['call_waiting_time', 'pickup_waiting_time'] + cols[idx+1:]
                seen = set()
                ordered = []
                for c in cols:
                    if c not in seen:
                        seen.add(c)
                        ordered.append(c)
                results_df = results_df[ordered]
            results_df.to_csv(output_file, index=False, encoding='utf-8-sig')
            print(f'   저장 완료: {output_file}')
            print(f'   총 서비스 기록: {len(results_df)}건')
            try:
                results_df['request_hour'] = pd.to_datetime('2000-01-01 ' + results_df['request_time']).dt.hour
                hourly_stats = results_df.groupby('request_hour').size()
                print(f'\n시간대별 서비스 완료 현황:')
                for hour in range(24):
                    count = hourly_stats.get(hour, 0)
                    if count > 0:
                        print(f'   {hour:2d}시: {count:3d}건')
            except Exception:
                pass
        else:
            # 비어 있어도 스키마를 가진 빈 CSV를 저장
            try:
                empty_cols = [
                    'demand_id','vehicle_id','vehicle_no','depot','request_time','assigned_time','pickup_time','dropoff_time',
                    'call_waiting_time','pickup_waiting_time','service_travel_time','total_trip_time',
                    'work_start','work_end','actual_work_hours',
                    'pickup_node_id','dropoff_node_id','pickup_x','pickup_y','dropoff_x','dropoff_y',
                    'pickup_depot_name','dropoff_depot_name'
                ]
                results_df = pd.DataFrame(columns=empty_cols)
                results_df.to_csv(output_file, index=False, encoding='utf-8-sig')
                print(f"   완료 기록 0건이지만 빈 결과 CSV 저장: {output_file}")
            except Exception as e:
                print(f'   결과 CSV 저장 스킵(빈 데이터, 오류: {e})')
        # 진행 로그 CSV 별도 저장
        try:
            if getattr(self, 'progress_log', None):
                df = pd.DataFrame(self.progress_log)
                base = os.path.splitext(os.path.basename(output_file))[0]
                progress_path = self._ensure_csv_path(os.path.join('results', f'{base}_progress'))
                df.to_csv(progress_path, index=False, encoding='utf-8-sig')
                print(f"   진행 로그 저장: {progress_path} ({len(df)}행)")
        except Exception as e:
            print(f"   진행 로그 저장 실패: {e}")
        return True


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--date', type=str, default='2025-06-23', help='Simulation date (YYYY-MM-DD)')
    parser.add_argument('--increasing', action='store_true', help='Enable scheduled vehicle increase (ON/OFF)')
    parser.add_argument('--schedule-csv', type=str, default='data/additional_depot_vehicles_schedule_template_v1.csv', help='Additional vehicles schedule CSV path')
    parser.add_argument('--adjust-schedule', action='store_true', help='Enable driver shift adjustment (ON/OFF)')
    parser.add_argument('--shift-rule', type=str, default=None, help="Generic shift rule like '6to4', '8to5'")
    parser.add_argument('--ratio', type=float, default=0.1, help='Share of drivers to adjust (0.0~1.0)')
    parser.add_argument('--lunch-duration', type=int, default=60, help='Lunch duration in minutes (default 60)')
    parser.add_argument('--lunch-realloc', type=str, default=None, help="Lunch reallocation only, e.g., '12->11:30=0.8,12->13=0.2'")
    # 관내/관외 및 권역 실험 옵션
    parser.add_argument('--force-both', action='store_true', help='Force all vehicles service_area to BOTH (100% BOTH scenario)')
    parser.add_argument('--region-strict-ratio', type=float, default=0.0, help='Ratio [0..1] to restrict assignment to same depot region')
    # always-on lunch breakdown; debug flags removed
    args = parser.parse_args()

    date_str = args.date
    schedule_csv = args.schedule_csv
    print(f'{date_str} 24시간 초단위 특별교통수단 시뮬레이션 (증차+근무시간 조정)')
    print('실시간 정밀 시뮬레이션 엔진')
    print('=' * 80)

    simulation = ScheduledIncreaseWithShiftSimulation()

    if not simulation.load_network():
        return False
    if not simulation.load_depot_info():
        return False
    if not simulation.load_vehicles():
        return False

    # 증차 스위치 ON일 때만 일정 기반 추가 차량 생성
    if args.increasing:
        if not simulation.load_additional_scheduled_vehicles(date_str, schedule_csv):
            return False
    # 관내외 겸용 100% 강제 적용
    simulation.force_both_service_area = bool(getattr(args, 'force_both', False))
    simulation.region_strict_ratio = max(0.0, min(1.0, float(getattr(args, 'region_strict_ratio', 0.0))))
    simulation.apply_force_both_service_area()

    simulation.load_hourly_speed_factors()

    if not simulation.load_accurate_schedules(date_str):
        return False

    # 근무시간 조정 적용
    applied_shift = False
    shift_tag = None
    if args.adjust_schedule and args.shift_rule:
        applied_shift = simulation.adjust_driver_shifts(args.shift_rule, args.ratio)
        if applied_shift:
            shift_tag = args.shift_rule.replace('->', 'to')

    # 점심시간 재배치만 적용
    applied_lunch = False
    lunch_tag = None
    realloc_rules = simulation._parse_lunch_realloc_arg(args.lunch_realloc)
    if realloc_rules:
        realloc_summary = simulation.adjust_lunch_reallocation(date_str, realloc_rules, duration_minutes=args.lunch_duration)
        moved_total = sum(sum(it['count'] for it in items) for items in realloc_summary.values()) if realloc_summary else 0
        applied_lunch = (moved_total > 0)
        if applied_lunch:
            tags = []
            for (sh, sm), items in realloc_rules.items():
                for it in items:
                    r_pct = int(it['ratio'] * 100)
                    if sm:
                        tags.append(f"{sh}:{sm}to{it['dst_h']}{(':' + str(it['dst_m'])) if it['dst_m'] else ''}_{r_pct}pct")
                    else:
                        tags.append(f"{sh}to{it['dst_h']}{(':' + str(it['dst_m'])) if it['dst_m'] else ''}_{r_pct}pct")
            if tags:
                lunch_tag = 'realloc_' + '_'.join(tags)
        # 스케줄-점심 정합 표준화
        simulation._normalize_schedule_with_lunch()

    # 전달: 디버그 옵션
    simulation.debug_lunch = bool(getattr(args, 'debug_lunch', False))
    simulation.debug_lunch_sample = int(getattr(args, 'debug_lunch_sample', 5))

    if not simulation.load_daily_demands(date_str):
        return False
    if not simulation.run_simulation(date_str):
        return False

    # 출력 파일명 구성
    date_suffix = date_str.replace('-', '')
    if args.increasing:
        base_name = os.path.basename(schedule_csv)
        m = re.search(r'(v\d+)', base_name)
        version_tag = m.group(1) if m else 'v0'
        parts = [f'results/scheduled_increase_with_shift_{version_tag}']
        if applied_shift and shift_tag:
            ratio_pct = int(max(0.0, min(1.0, float(args.ratio))) * 100)
            parts.append(f'{shift_tag}_{ratio_pct}pct')
        if applied_lunch and lunch_tag:
            parts.append(lunch_tag)
        if args.force_both:
            parts.append('forceBOTH')
        if simulation.region_strict_ratio > 0:
            parts.append(f'region{int(simulation.region_strict_ratio*100)}pct')
        parts.append(date_suffix)
        output_file = '_'.join(parts) + '.csv'
    else:
        parts = ['results/baseline_with_shift']
        if applied_shift and shift_tag:
            ratio_pct = int(max(0.0, min(1.0, float(args.ratio))) * 100)
            parts.append(f'{shift_tag}_{ratio_pct}pct')
        if applied_lunch and lunch_tag:
            parts.append(lunch_tag)
        if args.force_both:
            parts.append('forceBOTH')
        if simulation.region_strict_ratio > 0:
            parts.append(f'region{int(simulation.region_strict_ratio*100)}pct')
        parts.append(date_suffix)
        output_file = '_'.join(parts) + '.csv'

    # 확실히 .csv 확장자를 보장
    output_file = simulation._ensure_csv_path(output_file)
    # 파일명 위생 처리 및 .csv 확장자 보장
    output_file = simulation._finalize_output_path(output_file)
    simulation.save_results(output_file)
    print('\n초단위 시뮬레이션 완료!')
    return True


if __name__ == "__main__":
    main()


