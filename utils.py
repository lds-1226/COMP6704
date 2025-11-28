# utils.py
import pandas as pd
import networkx as nx
import random
from dataclasses import dataclass

@dataclass
class Flight:
    id: int
    arrival: float
    departure: float
    actype: str

@dataclass
class Gate:
    id: int
    compatible_types: list

def generate_kmg_instance(n_flights=200):
    # 简化版数据（真实作业你可以用你自己的航班表）
    flights = []
    for i in range(n_flights):
        arr = random.uniform(300, 1140)   # 5:00-19:00
        dep = arr + random.uniform(40, 180)
        f = Flight(i, arr, dep, random.choice(["A320","B738","A333"]))
        flights.append(f)
        
    gates = [Gate(i, ["A320","B738","A333"]) for i in range(83)]
    
    # 滑行道网络（218节点）
    G = nx.random_geometric_graph(218, 0.15, seed=42)
    pos = nx.get_node_attributes(G, 'pos')
    for u,v in G.edges():
        G[u][v]['weight'] = ((pos[u][0]-pos[v][0])**2 + (pos[u][1]-pos[v][1])**2)**0.5
        
    return flights, gates, G
