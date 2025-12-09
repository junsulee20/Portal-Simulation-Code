
import pickle
import networkx as nx
from pathlib import Path

def check_graph_size():
    path = Path("simulation/network/main_network_graph.pkl")
    if not path.exists():
        print(f"File not found: {path}")
        return

    try:
        with path.open("rb") as f:
            graph = pickle.load(f)
        
        print(f"Nodes: {graph.number_of_nodes()}")
        print(f"Edges: {graph.number_of_edges()}")
        
    except Exception as e:
        print(f"Error loading graph: {e}")

if __name__ == "__main__":
    check_graph_size()
