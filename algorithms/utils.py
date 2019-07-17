import networkx as nx
from algorithms.inner_types import NodeId, PointLL
from typing import List, Tuple


def project_nodes(g: nx.MultiDiGraph, locations: List[Tuple[float, float]]):
    import osmnx
    return set([osmnx.get_nearest_node(g, location) for location in locations])


def osm_to_pointll(g: nx.MultiDiGraph, node_id: NodeId) -> PointLL:
    return PointLL(lon=g.node[node_id]['x'], lat=g.node[node_id]['y'])
