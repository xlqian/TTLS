import networkx as nx
from algorithms.inner_types import NodeId, PointLL
from typing import List, Tuple


def project_nodes(g: nx.MultiDiGraph, locations: List[Tuple[float, float]]):
    import osmnx
    return set([osmnx.get_nearest_node(g, location) for location in locations])


def osm_to_pointll(g: nx.MultiDiGraph, node_id: NodeId) -> PointLL:
    return PointLL(lon=g.node[node_id]['x'], lat=g.node[node_id]['y'])


def get_path_length(g: nx.MultiDiGraph, path: List[NodeId]):
    it_1 = iter(path)
    it_2 = iter(path)
    next(it_2)
    return sum(g.adj[s][e][0]['length'] for s, e in zip(it_1, it_2))


def get_path_secs(g: nx.MultiDiGraph, path: List[NodeId], speed: float):
    return get_path_length(g, path) / speed
