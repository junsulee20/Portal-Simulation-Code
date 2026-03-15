# 배차 실패 원인 분석

## 🔍 배차 실패가 발생하는 조건

코드 분석 결과, 배차 실패는 다음 조건 중 하나라도 만족하면 발생합니다:

### 1. 차량 용량 초과
```python
if vehicle.onboard_passengers >= vehicle.capacity:
    continue
```
- **조건**: 차량에 이미 탑승 중인 승객 수가 용량(4명) 이상
- **현재 상황**: 로그상 모든 차량이 용량 이하이므로 이 조건은 아님

### 2. 경로 길이 제한 초과 (가장 가능성 높음)
```python
# assign_request()에서
if len(vehicle.path) > self.max_path_length:  # MAX_PATH_LENGTH = 18
    continue

# _find_best_insertion()에서
if path_len + 2 > self.max_path_length:  # 새 경로가 18 초과
    return None, math.inf
```
- **조건**: 차량의 현재 경로 길이가 18 stop을 초과하거나, 새 요청을 추가하면 18을 초과
- **현재 상황**: demand_018까지 성공했으므로, 이 시점에서 차량들의 경로가 이미 18에 가까웠을 가능성

### 3. 경로 계산 실패
```python
if math.isinf(original_path_time):
    continue

if math.isinf(new_path_time):
    continue
```
- **조건**: 경로 시간 계산 결과가 무한대 (경로가 존재하지 않음)
- **현재 상황**: 네트워크 그래프에서 경로가 없는 경우

### 4. 용량 제약 위반
```python
if not self._is_capacity_valid(path_candidate, vehicle.capacity, vehicle.onboard_passengers):
    continue
```
- **조건**: 삽입된 경로에서 차량 용량을 초과하는 지점이 존재
- **현재 상황**: 모든 후보 경로가 용량 제약을 위반

### 5. 모든 차량에서 배정 실패
```python
if best_vehicle is None:
    return None, [], math.inf
```
- **조건**: 모든 차량에 대해 위 조건들로 인해 배정 불가

## 📊 demand_019, demand_020 실패 시점 분석

### demand_018 시점의 차량 상태

**차량 1 경로** (demand_018 이후):
```
pickup:demand_008 → pickup:demand_011 → pickup:demand_006 → pickup:demand_002 
→ dropoff:demand_008 → pickup:demand_010 → dropoff:demand_010 
→ dropoff:demand_006 → dropoff:demand_002 → pickup:demand_003 
→ pickup:demand_009 → pickup:demand_014 → dropoff:demand_003 
→ dropoff:demand_009 → dropoff:demand_014 → dropoff:demand_011
```
**경로 길이: 16 stop**

**차량 2 경로** (demand_018 이후):
```
pickup:demand_001 → pickup:demand_005 → pickup:demand_013 
→ dropoff:demand_005 → pickup:demand_007 → pickup:demand_017 
→ dropoff:demand_013 → pickup:demand_018 → dropoff:demand_018 
→ pickup:demand_004 → dropoff:demand_007 → pickup:demand_016 
→ dropoff:demand_016 → pickup:demand_012 → dropoff:demand_004 
→ dropoff:demand_001 → dropoff:demand_017 → dropoff:demand_012
```
**경로 길이: 18 stop** (정확히 제한에 도달!)

## 🎯 배차 실패 원인

### 가장 가능성 높은 원인: **경로 길이 제한 초과**

1. **차량 2**: demand_018 배정 후 경로 길이가 정확히 18 stop에 도달
   - 새 요청을 추가하면 20 stop이 되어 제한(18) 초과
   - `path_len + 2 > self.max_path_length` 조건에 의해 배제

2. **차량 1**: demand_018 배정 후 경로 길이가 16 stop
   - 새 요청을 추가하면 18 stop이 되어 제한에 도달
   - 하지만 demand_019, demand_020의 경우 다른 이유로도 실패 가능

### 추가 가능한 원인

1. **용량 제약 위반**
   - 차량 1의 경로에 이미 많은 승객이 배정되어 있어, 새 요청을 추가하면 용량 초과 지점 발생

2. **경로 계산 실패**
   - 픽업/드롭오프 노드 간 경로가 존재하지 않거나 계산 불가

3. **후보 수 제한**
   - MAX_CANDIDATES_TO_EVALUATE = 80으로 제한되어 있어, 유효한 후보를 찾지 못함

## 💡 해결 방안

### 1. 경로 길이 제한 완화 (즉시 적용 가능)
```python
MAX_PATH_LENGTH = 18  # 현재
MAX_PATH_LENGTH = 25  # 완화
```
- **장점**: 더 많은 요청 배정 가능
- **단점**: 계산량 증가

### 2. 경로 길이 제한을 동적으로 조정
```python
# 요청 수에 따라 제한 조정
if num_requests > 15:
    max_path_length = 25
else:
    max_path_length = 18
```

### 3. 배정 실패 시 상세 정보 출력
```python
# 어떤 조건 때문에 실패했는지 출력
if len(vehicle.path) > self.max_path_length:
    print(f"  [차량 {vehicle.vehicle_id}] 경로 길이 제한 초과: {len(vehicle.path)} > {self.max_path_length}")
```

### 4. 경로 재구성 (고급)
- 경로가 길어지면 일부 승객을 다른 차량으로 재배정
- 경로 최적화 알고리즘 적용

## 📝 결론

**demand_019와 demand_020이 배정 실패한 주요 원인**:
1. **경로 길이 제한 초과** (가장 가능성 높음)
   - 차량 2: 이미 18 stop에 도달
   - 차량 1: 16 stop이지만 새 요청 추가 시 제한 초과 가능
2. **용량 제약 위반** (가능성 있음)
   - 경로가 길어지면서 용량 제약을 만족하는 삽입 위치가 없음

**즉시 개선 가능한 방법**:
- MAX_PATH_LENGTH를 18 → 25로 증가
- 배정 실패 시 상세 원인 출력 추가

