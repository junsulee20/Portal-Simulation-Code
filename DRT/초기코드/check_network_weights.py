"""
네트워크 그래프 파일의 가중치(weight) 속성 확인 스크립트
"""
import pickle
import networkx as nx
from pathlib import Path

def check_network_weights():
    """네트워크 그래프의 가중치 속성을 확인합니다."""
    # 현재 스크립트 위치 기준으로 경로 설정
    script_dir = Path(__file__).parent.parent
    graph_path = script_dir / "simulation" / "network" / "main_network_graph.pkl"
    
    if not graph_path.exists():
        print(f"❌ 파일을 찾을 수 없습니다: {graph_path}")
        return
    
    try:
        print("=" * 70)
        print("네트워크 그래프 가중치 확인")
        print("=" * 70)
        print()
        
        # 그래프 로드
        with graph_path.open("rb") as f:
            graph = pickle.load(f)
        
        print(f"✅ 그래프 타입: {type(graph)}")
        print(f"✅ 노드 수: {graph.number_of_nodes():,}개")
        print(f"✅ 간선 수: {graph.number_of_edges():,}개")
        print()
        
        # 간선 속성 확인
        sample_edges = list(graph.edges(data=True))[:10]
        print("📊 샘플 간선 데이터 (처음 10개):")
        for u, v, data in sample_edges:
            print(f"   {u} -> {v}: {data}")
        print()
        
        # 모든 간선 속성 수집
        all_attrs = set()
        for u, v, data in graph.edges(data=True):
            all_attrs.update(data.keys())
        
        print(f"📋 간선에 존재하는 모든 속성: {sorted(all_attrs)}")
        print()
        
        # weight 속성 확인
        has_weight = "weight" in all_attrs
        print(f"🔍 'weight' 속성 존재 여부: {'✅ 예' if has_weight else '❌ 아니오'}")
        print()
        
        if has_weight:
            # weight 값 수집
            weights = []
            edges_with_weight = 0
            edges_without_weight = 0
            
            for u, v, data in graph.edges(data=True):
                if "weight" in data:
                    weight_val = data["weight"]
                    if weight_val is not None:
                        weights.append(weight_val)
                        edges_with_weight += 1
                    else:
                        edges_without_weight += 1
                else:
                    edges_without_weight += 1
            
            if weights:
                print(f"✅ weight를 가진 간선 수: {edges_with_weight:,}개")
                print(f"❌ weight가 없는 간선 수: {edges_without_weight:,}개")
                print()
                print(f"📊 Weight 통계:")
                print(f"   최소값: {min(weights):.6f}")
                print(f"   최대값: {max(weights):.6f}")
                print(f"   평균값: {sum(weights)/len(weights):.6f}")
                print(f"   중앙값: {sorted(weights)[len(weights)//2]:.6f}")
                print()
                print(f"📋 샘플 weight 값 (처음 20개):")
                for i, w in enumerate(weights[:20], 1):
                    print(f"   {i:2d}. {w:.6f}")
                
                # weight 단위 추정 (코드에서 분 단위로 가정하고 있음)
                print()
                print("💡 Weight 단위 추정:")
                print("   - 코드에서 weight를 '분(minutes)' 단위로 가정하고 있음")
                print("   - travel_seconds() 메서드에서 minutes * 60.0으로 변환")
                print("   - 따라서 weight는 도로 통행 시간(분)을 나타냄")
            else:
                print("⚠️  weight 속성이 있지만 값이 None인 간선만 존재합니다.")
        else:
            print("❌ 'weight' 속성이 존재하지 않습니다!")
            print("   최단 경로 계산 시 weight를 사용할 수 없습니다.")
        
        print()
        print("=" * 70)
        print("결론:")
        if has_weight and weights:
            print("✅ 네트워크 그래프에 도로별 가중치(weight)가 존재합니다!")
            print("✅ 가중치는 도로 통행 시간(분)을 나타냅니다.")
            print("✅ 목적함수를 '거리 증가'에서 '시간 증가'로 변경하는 것이 타당합니다.")
        else:
            print("❌ 네트워크 그래프에 가중치가 없거나 유효하지 않습니다.")
        print("=" * 70)
        
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_network_weights()

