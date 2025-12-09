
import pickle
import networkx as nx
import math
import time
import random

def analyze_graph():
    try:
        print("Loading graph...")
        with open("simulation/network/main_network_graph.pkl", "rb") as f:
            graph = pickle.load(f)
        
        print(f"Nodes: {len(graph.nodes)}")
        print(f"Edges: {len(graph.edges)}")
        
        # Check coordinates
        nodes_with_coords = [n for n in graph.nodes if 'longitude' in graph.nodes[n] and 'latitude' in graph.nodes[n]]
        print(f"Nodes with coordinates: {len(nodes_with_coords)} / {len(graph.nodes)}")
        
        if not nodes_with_coords:
            print("No coordinates found. Cannot use A*.")
            return

        # Check weights
        weights = [d['weight'] for u, v, d in graph.edges(data=True) if 'weight' in d]
        if not weights:
            print("No weights found.")
            return
            
        print(f"Min weight: {min(weights)}")
        print(f"Max weight: {max(weights)}")
        print(f"Avg weight: {sum(weights)/len(weights)}")
        
        # Estimate speed for heuristic
        # Speed = Distance / Time
        # We need a heuristic h(u, v) <= d(u, v) (admissible)
        # Time = Weight. Distance = Euclidean (or Haversine).
        # So we need max(Distance / Weight) to find the "fastest" possible speed in the graph.
        # Then h(u, v) = Distance(u, v) / Max_Speed <= Actual_Time
        
        max_speed = 0.0
        
        for u, v, d in graph.edges(data=True):
            if 'weight' not in d or d['weight'] <= 0:
                continue
            
            u_data = graph.nodes[u]
            v_data = graph.nodes[v]
            
            if 'longitude' not in u_data or 'longitude' not in v_data:
                continue
                
            dx = u_data['longitude'] - v_data['longitude']
            dy = u_data['latitude'] - v_data['latitude']
            dist = math.sqrt(dx*dx + dy*dy)
            
            speed = dist / d['weight']
            if speed > max_speed:
                max_speed = speed
                
        print(f"Estimated Max Speed (deg/min): {max_speed}")
        
        # Test A* vs Dijkstra
        print("\nTesting Performance (10 random pairs)...")
        
        def heuristic(u, v):
            u_data = graph.nodes[u]
            v_data = graph.nodes[v]
            dx = u_data['longitude'] - v_data['longitude']
            dy = u_data['latitude'] - v_data['latitude']
            dist = math.sqrt(dx*dx + dy*dy)
            return dist / max_speed if max_speed > 0 else 0

        total_dijkstra_time = 0
        total_astar_time = 0
        
        test_pairs = []
        nodes_list = list(graph.nodes)
        for _ in range(10):
            u = random.choice(nodes_list)
            v = random.choice(nodes_list)
            test_pairs.append((u, v))
            
        for u, v in test_pairs:
            start = time.time()
            try:
                nx.shortest_path_length(graph, u, v, weight='weight')
            except:
                pass
            total_dijkstra_time += time.time() - start
            
            start = time.time()
            try:
                nx.astar_path_length(graph, u, v, heuristic=heuristic, weight='weight')
            except:
                pass
            total_astar_time += time.time() - start
            
        print(f"Dijkstra Avg Time: {total_dijkstra_time/10:.6f}s")
        print(f"A* Avg Time:       {total_astar_time/10:.6f}s")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    analyze_graph()
