# DRT 배정 알고리즘 분석 및 개선 방안

## 🔍 현재 알고리즘 방식

### 알고리즘 유형: **완전 탐색 기반 최소 삽입 비용 (Brute-Force Minimum Insertion Cost)**

현재 구현된 알고리즘은 **모든 가능한 삽입 위치 조합을 탐색**하는 완전 탐색 방식입니다.

### 동작 과정

#### 1단계: 차량 순회
```python
for vehicle in vehicles:
    # 각 차량에 대해 최적 삽입 위치 탐색
```

#### 2단계: 모든 삽입 위치 조합 생성 (완전 탐색)
```python
for pickup_index in range(path_len + 1):  # 픽업 위치: 0 ~ path_len
    for dropoff_index in range(pickup_index + 1, len(path_with_pickup) + 1):  # 드롭오프 위치: pickup_index+1 ~ 끝
        # 모든 조합에 대해 비용 계산
```

**예시**: 경로 길이가 10 stop인 경우
- 픽업 위치: 11개 (0~10)
- 각 픽업 위치마다 드롭오프 위치: 평균 6개
- **총 후보 수: 약 66개** (11 × 6)

#### 3단계: 각 후보에 대한 비용 계산
```python
# 1. 경로 시간 계산
new_path_time = self._calculate_path_time(vehicle.current_node, path_candidate)

# 2. 비용 계산
cost_increase = new_path_time - original_path_time
final_cost = (W_COST_INCREASE * cost_increase) + (W_PATH_LENGTH * new_path_time)
```

**비용 함수**:
```
최종 비용 = 0.7 × 시간 증가량 + 0.3 × 새 경로 총 시간
```

#### 4단계: 최적 차량 선택
```python
if candidate_cost < min_final_cost:
    best_vehicle = vehicle
    best_new_path = candidate_path
    min_final_cost = candidate_cost
```

### 현재 최적화 적용 사항

1. **경로 길이 제한**: MAX_PATH_LENGTH = 18
2. **후보 수 제한**: MAX_CANDIDATES_TO_EVALUATE = 80
3. **조기 종료**: 현재 최적해의 1.3배 이상인 후보는 건너뛰기
4. **증분 계산**: 경로가 길 때 전체 재계산 대신 증가분만 계산

### 복잡도 분석

- **시간 복잡도**: O(n² × m × k)
  - n: 경로 길이 (stop 개수)
  - m: 차량 수
  - k: 각 경로 계산 비용 (최단 경로 알고리즘)

- **공간 복잡도**: O(n²)
  - 각 후보마다 경로 복사 필요

### 문제점

1. **O(n²) 복잡도**: 경로가 길어질수록 후보 수가 제곱으로 증가
2. **모든 후보 평가**: 유망하지 않은 후보도 모두 평가
3. **중복 계산**: 비슷한 경로에 대해 반복적으로 계산
4. **확장성 부족**: 요청이 많아질수록 급격히 느려짐

---

## 💡 개선 방안

### 방안 1: 휴리스틱 기반 순차적 삽입 (Heuristic Sequential Insertion)

#### 개념
완전 탐색 대신, **픽업과 드롭오프를 순차적으로 최적 위치에 삽입**하는 방식

#### 동작 과정

```python
# 1단계: 픽업 위치만 최적화
best_pickup_index = None
min_pickup_cost = inf

for pickup_index in range(path_len + 1):
    # 픽업만 삽입한 경로의 비용 계산
    cost = calculate_cost_with_pickup_only(pickup_index)
    if cost < min_pickup_cost:
        best_pickup_index = pickup_index
        min_pickup_cost = cost

# 2단계: 최적 픽업 위치 고정 후 드롭오프 위치만 최적화
best_dropoff_index = None
min_dropoff_cost = inf

for dropoff_index in range(best_pickup_index + 1, len(path_with_pickup) + 1):
    # 드롭오프 삽입한 경로의 비용 계산
    cost = calculate_cost_with_dropoff(dropoff_index)
    if cost < min_dropoff_cost:
        best_dropoff_index = dropoff_index
        min_dropoff_cost = cost
```

#### 장점
- **복잡도 감소**: O(n²) → O(n)
- **속도 향상**: 후보 수가 n² → 2n으로 감소
- **실용적**: 대부분의 경우 충분히 좋은 해를 찾음

#### 단점
- **전역 최적해 미보장**: 순차적 선택이므로 최적해를 놓칠 수 있음
- **로컬 최적해**: 지역적으로 최선이지만 전체적으로는 아닐 수 있음

#### 예상 성능 향상
- **10배 이상 속도 향상** (n² → n)
- 요청 20개 기준: 약 36,000번 → 약 3,600번의 계산

---

### 방안 2: 휴리스틱 기반 후보 필터링 (Heuristic Candidate Filtering)

#### 개념
모든 후보를 평가하지 않고, **유망한 후보만 선별하여 평가**

#### 필터링 기준

1. **거리 기반 필터링**
   ```python
   # 픽업/드롭오프 지점과 가까운 기존 stop 주변만 고려
   nearby_stops = find_nearby_stops(pickup_node, max_distance=5km)
   pickup_candidates = [i for i in range(path_len+1) 
                       if path[i] is near pickup_node]
   ```

2. **방향 기반 필터링**
   ```python
   # 차량의 진행 방향과 일치하는 위치만 고려
   if is_same_direction(vehicle_direction, pickup_direction):
       # 후보에 포함
   ```

3. **비용 하한선 기반 필터링**
   ```python
   # 현재 최적해보다 나쁠 가능성이 높은 후보는 제외
   estimated_cost = estimate_insertion_cost(pickup_index, dropoff_index)
   if estimated_cost > best_cost * 1.5:
       continue  # 건너뛰기
   ```

#### 장점
- **정확도 유지**: 완전 탐색과 동일한 최적해 보장 가능
- **속도 향상**: 평가할 후보 수 감소
- **유연성**: 필터링 기준 조정 가능

#### 단점
- **구현 복잡도**: 필터링 로직 추가 필요
- **파라미터 튜닝**: 필터링 기준 최적화 필요

#### 예상 성능 향상
- **3-5배 속도 향상** (후보 수 80 → 20-30개)
- 요청 20개 기준: 약 36,000번 → 약 7,200-10,800번의 계산

---

### 방안 3: 경로 시간 캐싱 강화 (Enhanced Path Time Caching)

#### 개념
**부분 경로의 시간을 캐싱**하여 중복 계산 제거

#### 구현 방법

```python
class PathTimeCache:
    def __init__(self):
        self._cache = {}  # (start_node, stop_sequence_hash) -> time
    
    def get_path_time(self, start_node, path):
        # 경로의 해시값 생성
        path_hash = hash(tuple((s.node_id, s.stop_type) for s in path))
        key = (start_node, path_hash)
        
        if key in self._cache:
            return self._cache[key]
        
        # 계산 후 캐싱
        time = calculate_path_time(start_node, path)
        self._cache[key] = time
        return time
```

#### 장점
- **중복 계산 제거**: 동일한 경로는 한 번만 계산
- **점진적 개선**: 시뮬레이션이 진행될수록 캐시 히트율 증가
- **구현 간단**: 기존 코드에 캐시 레이어 추가

#### 단점
- **메모리 사용 증가**: 캐시 저장 공간 필요
- **캐시 관리**: 오래된 캐시 정리 필요

#### 예상 성능 향상
- **20-30% 속도 향상** (중복 계산 제거)
- 요청 20개 기준: 약 36,000번 → 약 25,200-28,800번의 계산

---

### 방안 4: 하이브리드 접근법 (Hybrid Approach)

#### 개념
**여러 최적화 기법을 조합**하여 성능과 정확도의 균형

#### 조합 전략

1. **경로가 짧을 때 (≤5 stop)**: 완전 탐색
   - 후보 수가 적어 완전 탐색이 빠름
   - 정확한 최적해 보장

2. **경로가 중간일 때 (6-10 stop)**: 휴리스틱 필터링
   - 유망한 후보만 선별하여 평가
   - 정확도와 속도의 균형

3. **경로가 길 때 (>10 stop)**: 순차적 삽입
   - O(n) 복잡도로 빠른 계산
   - 실용적인 해 제공

#### 구현 예시

```python
def _find_best_insertion(self, vehicle, request, original_path_time):
    path_len = len(vehicle.path)
    
    if path_len <= 5:
        # 완전 탐색 (후보 수가 적음)
        return self._brute_force_search(vehicle, request, original_path_time)
    elif path_len <= 10:
        # 휴리스틱 필터링
        return self._heuristic_filtered_search(vehicle, request, original_path_time)
    else:
        # 순차적 삽입
        return self._sequential_insertion(vehicle, request, original_path_time)
```

#### 장점
- **최적 성능**: 상황에 맞는 최적 알고리즘 선택
- **정확도 보장**: 짧은 경로에서는 최적해 보장
- **확장성**: 경로가 길어져도 성능 유지

#### 예상 성능 향상
- **5-10배 속도 향상** (상황에 따라 다름)
- 요청 20개 기준: 약 36,000번 → 약 3,600-7,200번의 계산

---

## 📊 비교표

| 방식 | 복잡도 | 정확도 | 구현 난이도 | 예상 속도 향상 |
|------|--------|--------|------------|---------------|
| **현재 (완전 탐색)** | O(n²) | 최적해 보장 | 낮음 | 1x (기준) |
| **순차적 삽입** | O(n) | 근사해 | 중간 | 10x |
| **휴리스틱 필터링** | O(n²) | 최적해 보장 | 높음 | 3-5x |
| **경로 캐싱** | O(n²) | 최적해 보장 | 낮음 | 1.2-1.3x |
| **하이브리드** | O(n)~O(n²) | 상황별 | 높음 | 5-10x |

---

## 🎯 권장 개선 방안

### 단기 개선 (빠른 구현)
1. **후보 수 제한 강화**: 80 → 50
2. **경로 시간 재사용**: 배정 단계에서 계산한 시간을 통행시간 계산에 재사용
3. **경로 길이 제한 강화**: 18 → 15

**예상 효과**: 30-50% 속도 향상

### 중기 개선 (균형잡힌 접근)
1. **하이브리드 접근법 구현**
   - 경로 길이에 따라 알고리즘 선택
   - 짧은 경로: 완전 탐색
   - 긴 경로: 순차적 삽입

**예상 효과**: 5-10배 속도 향상

### 장기 개선 (최적 성능)
1. **고급 휴리스틱 필터링**
   - 거리/방향 기반 후보 선별
   - 비용 하한선 기반 조기 종료

2. **경로 시간 캐싱 강화**
   - 부분 경로 캐싱
   - LRU 캐시 관리

**예상 효과**: 10-20배 속도 향상

---

## 💻 구현 우선순위

1. **우선순위 1**: 경로 시간 재사용 (가장 쉬움, 즉시 효과)
2. **우선순위 2**: 하이브리드 접근법 (균형잡힌 개선)
3. **우선순위 3**: 휴리스틱 필터링 (최대 성능)

