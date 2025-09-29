# scheduled_increasing_with_shift_scenario_simulation 시뮬레이션 안내

본 문서는 `scheduled_increasing_with_shift_scenario_simulation.py` 단일 스크립트 사용과 동작만 다룹니다.

## 개요
- **목적**: 초단위(1초 tick)로 24시간 시뮬레이션을 수행하여 대기/서비스/총 소요시간 등을 산출
- **엔진**: NetworkX 최단경로, 시간대별 속도계수, 차량/승객 상태머신, 즉시배정/정규배정
- **시간 해상도**: 1초(하루 86,400 tick)
- **주요 규칙**
  - 승차 3분, 하차 2분 반영
  - 즉시배정 시 `assigned_time`/`call_waiting_time` 일관 반영
  - 전일-당일 연속 운행(야간 보장)

## 필요 파일
- `network/main_network_graph.pkl`
- `network/depot_main_network_mapping_fixed.csv`
- `network/fixed_vehicle_mapping_63.csv`
- `data/demand_main_network_mapped.csv`
- 선택: `data/hourly_speed_factors.csv`, `network/accurate_individual_vehicle_schedule.csv`
- 선택(증차): `data/additional_depot_vehicles_schedule_template_v1.csv`

## 옵션 및 기능
- 공통: `--date YYYY-MM-DD`
- 일정 기반 증차: `--increasing`, `--schedule-csv data/additional_depot_vehicles_schedule_template_v1.csv`
- 근무시간 조정: `--adjust-schedule`, `--shift-rule "XtoY"`, `--ratio p`
- 점심 재배치: `--lunch-realloc "12->11:30=0.8,12->13=0.2"`, `--lunch-duration 60`
- 관내/관외 시나리오(겸용 100%): `--force-both`
- 권역 고정배차 시나리오(비율): `--region-strict-ratio p` (0~1, 동일 권역 차량만으로 시도할 비율)

## 배정/운행 규칙
- 관내/관외 구분
  - `origin1`/`destination1` 중 하나라도 `경기도 화성시`가 아니면 관외(`is_outside_area=True`)
  - 관외는 `BOTH` 차량만 배정, 관내는 `INSIDE_ONLY`/`BOTH` 허용
- 권역(설명)
  - 결과에 `pickup_depot_name`/`dropoff_depot_name` 포함
  - 기본은 ETA 최단 우선(동일 권역 고정배차는 기본 미적용)
- 스케줄 단절 보호: 다음 활성 시간 시작 전 완료 불가 시 배정 제외
- 속도계수: `hourly_speed_factors.csv`로 시간대별 이동시간 스케일링
- 점심 재배치
  - 소스 시각 포함 창 또는 스케줄 갭(False이고 양옆 True)을 점심으로 간주해 재배치
  - 단일 타깃은 비율만큼만 이동, 다중 타깃은 마지막 타깃이 잔여 수용
- 진행 로그: 시간대별 가동/가용/서비스중/대기/점심/추가차량 집계, 별도 CSV 저장

## 관내/관외 · 권역 실행 안내
- 별도의 실행 스위치 없이 항상 적용됩니다(관외는 `BOTH` 차량만 배정).
- 실행 예시는 일반 실행과 동일하며, 결과 CSV의 `pickup_depot_name`/`dropoff_depot_name`(권역)으로 분석할 수 있습니다.

## 실행 예시(한 줄)
- 모든 기능 OFF:
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23
```
- 증차 ON:
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23 --increasing --schedule-csv data/additional_depot_vehicles_schedule_template_v1.csv
```
- 근무시간 조정(예: 6→4, 30%):
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23 --adjust-schedule --shift-rule "6to4" --ratio 0.3
```
- 점심 재배치(예: 12시 중 80%를 11:30으로):
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23 --lunch-realloc "12->11:30=0.8" --lunch-duration 60
```
- 관내/관외/권역 분석 실행(예: 결과에서 관외만 필터링):
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23
# 결과 CSV에서 is_outside_area==True, pickup_depot_name / dropoff_depot_name 기준으로 분석
```
- 관내/관외 시나리오(겸용 100%):
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23 --force-both
```
- 권역 고정배차(동일 권역 70% 우선):
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23 --region-strict-ratio 0.7
```
- 증차+근무시간+점심 동시:
```bash
python scheduled_increasing_with_shift_scenario_simulation.py --date 2025-06-23 --increasing --schedule-csv data/additional_depot_vehicles_schedule_template_v1.csv --adjust-schedule --shift-rule "6to4" --ratio 0.3 --lunch-realloc "12->11:30=0.8,12->13=0.2" --lunch-duration 60
```

## 시나리오별 출력 파일 패턴
- 공통: 결과는 `results/`에 저장, 진행 로그는 같은 접두사에 `_progress.csv` 접미사로 저장
- 모든 기능 OFF(증차 OFF, 근무/점심 OFF):
  - 결과: `results/baseline_with_shift_YYYYMMDD.csv`
  - 진행: `results/baseline_with_shift_YYYYMMDD_progress.csv`
- 증차 ON(예: 스케줄 파일명이 v1 포함):
  - 결과: `results/scheduled_increase_with_shift_v1_YYYYMMDD.csv`
  - 진행: `results/scheduled_increase_with_shift_v1_YYYYMMDD_progress.csv`
- 근무시간 조정만(예: `6to4`, 30%):
  - 결과: `results/baseline_with_shift_6to4_30pct_YYYYMMDD.csv`
  - 진행: `results/baseline_with_shift_6to4_30pct_YYYYMMDD_progress.csv`
- 점심 재배치만(예: `12->11:30=0.8`):
  - 결과: `results/baseline_with_shift_realloc_12to11:30_80pct_YYYYMMDD.csv` (실제 태그는 규칙에 따라 구성)
  - 진행: 동일 접두사 + `_progress.csv`
- 증차+근무시간+점심 동시(예: v1 + 6to4 30% + 재배치):
  - 결과: `results/scheduled_increase_with_shift_v1_6to4_30pct_realloc_..._YYYYMMDD.csv`
  - 진행: 동일 접두사 + `_progress.csv`
- 관내/관외(겸용 100%): 접두사에 `forceBOTH` 추가
  - 예) `results/baseline_with_shift_forceBOTH_YYYYMMDD.csv`
- 권역 고정배차(비율 적용): 접두사에 `region{pct}pct` 추가
  - 예) `results/baseline_with_shift_region70pct_YYYYMMDD.csv`

### 태깅 결합 규칙(접두사 구성 순서)
- 기본 접두사: `baseline_with_shift` 또는 `scheduled_increase_with_shift_{vX}`
- 그 뒤에 선택 태그가 추가됩니다(존재 시):
  1) `forceBOTH` (겸용 100%)
  2) `region{pct}pct` (권역 비율)
  3) `{shiftTag}_{ratioPct}pct` (근무시간 조정)
  4) `realloc_{...}` (점심 재배치 요약)
- 마지막에 `_YYYYMMDD.csv`가 붙습니다. 진행 로그는 동일 접두사에 `_progress.csv`로 저장됩니다.
- 파일명은 내부적으로 무효 문자를 `_`로 치환하고, 확장자 `.csv`를 강제 보장합니다.

### 결합 예시
- `results/scheduled_increase_with_shift_v1_forceBOTH_region50pct_6to4_30pct_realloc_12to11:30_80pct_20250623.csv`
- 진행 로그: `results/scheduled_increase_with_shift_v1_forceBOTH_region50pct_6to4_30pct_realloc_12to11:30_80pct_20250623_progress.csv`

## 월간 실행(배치)
- 스크립트: `run_month_simulations.py`
- 특징: 지정 월의 모든 날짜를 순회 실행, 자식 프로세스의 상세 로그는 콘솔에 출력하지 않음(숨김), 결과/진행 로그는 일자별 CSV 저장
- 전달 옵션: 일일 스크립트와 동일한 주요 옵션을 그대로 전달
  - `--increasing`, `--schedule-csv`, `--adjust-schedule`, `--shift-rule`, `--ratio`, `--lunch-realloc`, `--lunch-duration`, `--force-both`, `--region-strict-ratio`
- 예시(증차+근무시간+점심 동시):
```bash
python run_month_simulations.py --year 2025 --month 6 --script scheduled --increasing --schedule-csv data/additional_depot_vehicles_schedule_template_v1.csv --adjust-schedule --shift-rule "6to4" --ratio 0.3 --lunch-realloc "12->11:30=0.8,12->13=0.2" --lunch-duration 60
```
- 예시(관내/관외 100% 겸용):
```bash
python run_month_simulations.py --year 2025 --month 6 --script scheduled --force-both
```
- 예시(권역 고정배차 70%):
```bash
python run_month_simulations.py --year 2025 --month 6 --script scheduled --region-strict-ratio 0.7
```
-
예시(모든 기능 OFF, 스케줄드 엔진 사용):
```bash
python run_month_simulations.py --year 2025 --month 6 --script scheduled
```
- 출력 위치: 각 일자별로 `results/`에 일일 결과/진행 로그 CSV가 생성됩니다.
- 콘솔 출력: 성공/실패 요약만 표시되며, 각 일일 실행의 상세 진행 로그는 표시되지 않습니다.

## 산출물
- 결과 CSV: 주요 시간/대기/서비스 지표 + 승하차 노드/좌표/권역 포함
- 진행 로그 CSV: `_progress.csv` 접미사로 저장

## 시뮬레이션 코드 구조(상세)
- 파일: `simulation/scheduled_increasing_with_shift_scenario_simulation.py`

### 핵심 클래스/상태
- VehicleStatus: `IDLE`, `ASSIGNED`, `TRAVELING_TO_PICKUP`, `PICKING_UP`, `TRAVELING_TO_DROPOFF`, `DROPPING_OFF`, `RETURNING`, `OFF_DUTY`
- PassengerStatus: `REQUESTED`, `ASSIGNED`, `PICKED_UP`, `DROPPED_OFF`, `CANCELLED`
- Location: `node_id`
- Vehicle
  - 식별/기본: `vehicle_id`, `vehicle_no`, `depot_name`, `depot_location`
  - 서비스: `service_area`(INSIDE_ONLY/BOTH), `status`, `assigned_passenger`
  - 근무: `accurate_schedule{0..23}`, `work_start`, `work_end`, `actual_work_hours`, `lunch_windows`(점심창 목록)
  - 집계: `daily_services`, `total_distance`, `total_service_time`
- Passenger
  - 식별: `demand_id`, `customer_id`
  - 시간/위치: `request_time`, `pickup_location`, `dropoff_location`
  - 상태/지표: `status`, `assigned_time`, `pickup_time`, `dropoff_time`, `call_waiting_time`, `pickup_waiting_time`, `service_travel_time`, `total_trip_time`
  - 권역: `pickup_depot_name`, `dropoff_depot_name`

### 시뮬레이터 속성
- `vehicles`, `passengers`, `network_graph`, `depot_info`
- 대기열/배정: `pending_passengers`, `assigned_demands`
- 결과/로그: `service_records`, `demand_call_log`, `vehicle_service_log`, `progress_log`
- 경로/성능: `path_cache`, `hourly_speed_factors`, `base_speed_factor_assumed`
- 실험옵션: `force_both_service_area`, `region_strict_ratio`

### 주요 로딩 함수
- `load_network()`: NetworkX 그래프 로드(가중치=분)
- `load_depot_info()`: 차고지→노드 매핑
- `load_vehicles()`: 기본 63대 로드(차고지별 INSIDE/BOTH 할당), 스케줄 맵 초기화
- `load_additional_scheduled_vehicles(date, csv)`: 일정 템플릿을 읽어 시간대 활성 추가 차량 생성
- `load_accurate_schedules(date)`: 날짜별 스케줄 로드, end-exclusive 보정, 전일 연속운행 처리
- `load_hourly_speed_factors(csv)`: 시간대별 속도계수 로드
- `load_daily_demands(date)`: 당일 수요만 필터, `origin1/destination1`로 관외 여부 산정, 권역명 주입

### 배정/상태머신
- `assign_passenger_to_vehicle(passenger, now)`
  - 후보 필터: 근무 시간, 점심창(IDLE·미배정 제외), 관외 수요시 INSIDE_ONLY 제외, 끝시간 임박/다음 시간 비활성 보호
  - 권역 비율: `region_strict_ratio` 확률로 동일 권역 차량만 후보로 제한
  - 선택: ETA 최단 차량
  - 설정: `assigned_time`, `call_waiting_time` 기록, 차량 상태/종료 예상시간 설정
- `process_pending_passengers(now)`
  - 대기열 승객을 유사 규칙으로 즉시배정, 로그/대기열 관리
- `update_vehicle_status(now)`
  - 시간대 근무 여부에 따른 `OFF_DUTY/IDLE` 전이
  - ASSIGNED→TRAVELING_TO_PICKUP(이동시간 산정)
  - TRAVELING_TO_PICKUP→PICKING_UP(픽업 시각/대기 기록, 승차 3분)
  - PICKING_UP→TRAVELING_TO_DROPOFF(서비스 이동)
  - TRAVELING_TO_DROPOFF→DROPPING_OFF(하차 2분)
  - DROPPING_OFF→IDLE/OFF_DUTY(서비스 기록 적재, 복귀)

### 실행/진행 로그
- `run_simulation(date)`
  - 24시간(초단위) 루프, 5분 간격 진행 현황 출력/적재
  - 집계: 가동/가용/서비스중, 대기(미배정/차량대기), 점심(총/IDLE제외/운행중), 추가차량(총/가동)

### 결과 저장
- `save_results(output_file)`
  - 메인 결과: 시간/대기/서비스 지표 + 승하차 노드/좌표 + 권역명
  - 컬럼 재배치: `dropoff_time` 뒤에 `call_waiting_time`, `pickup_waiting_time`
  - 진행 로그: 같은 접두사의 `_progress.csv`

### 옵션/파일명 태깅
- 증차: `scheduled_increase_with_shift_{vX}` 접두사
- 근무시간: `{shiftTag}_{ratioPct}pct`
- 점심 재배치: `realloc_...`
- 겸용 100%: `forceBOTH`
- 권역 비율: `region{pct}pct`

### 배정 프로세스 플로우(텍스트 다이어그램)
1) 승객 요청 도착(now)
   - 대기열 삽입 혹은 즉시 배정 시도
2) 후보 차량 수집
   - 근무 시간(`accurate_schedule[now.hour] == True`)인 차량만
   - 현재 점심창에 포함되면(AND IDLE AND 미배정) 신규 배정 제외
   - 차량 종료 임박/다음 시간 비활성 등 보호 규칙으로 제외
3) 관내/관외 필터
   - 관외 수요이면 `INSIDE_ONLY` 차량 제외, `BOTH`만 유지
4) 권역 고정배차(확률)
   - `random() < region_strict_ratio`이면 픽업 권역과 동일한 차량만 후보 유지
   - 동일 권역이 없으면 전체 후보 유지로 폴백
5) 후보에서 ETA 최단 차량 선택
   - NetworkX 최단경로 기반 이동시간 추정(시간대 속도계수 반영)
6) 배정 확정
   - 승객 `assigned_time`, `call_waiting_time` 기록
   - 차량 상태 `ASSIGNED`→`TRAVELING_TO_PICKUP` 전이, 도착 예정시간 설정
7) 상태 전이 루틴
   - 픽업 도착→`PICKING_UP`(3분)
   - 승차 완료→`TRAVELING_TO_DROPOFF`
   - 하차 도착→`DROPPING_OFF`(2분)→서비스 기록 저장→`IDLE` 또는 `OFF_DUTY`

### 시간 진행 루프(텍스트 다이어그램)
1) 매 초(now = 00:00:00 → 23:59:59)
2) 차량 상태 업데이트(도착/전이/완료 처리)
3) 새로 도착한 수요를 즉시 배정 시도
4) 배정 실패 수요는 대기열로 유지
5) 5분 간격으로 진행 로그 스냅샷 수집/출력
