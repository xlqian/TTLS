from typing import Set, Callable, Dict
import networkx as nx

from algorithms.isochrone import Isocrhone
from algorithms.inner_types import NodeId, Cost
from call_backs import astar_call_back
WALKING_SPEED = 1.4
BIKE_SPEED = 3.3


class MultiModalIsochrone(object):
    _bss_nodes: Set[NodeId]
    _first_isochrone: Isocrhone
    _second_isochrone: Isocrhone
    _third_isochrone: Isocrhone

    def __init__(self, bss_nodes: Set[NodeId], walking_speed: float=WALKING_SPEED, bike_speed: float=BIKE_SPEED):
        self._bss_nodes = bss_nodes
        self._first_isochrone = Isocrhone(speed=walking_speed)
        self._second_isochrone = Isocrhone(speed=bike_speed)
        self._third_isochrone = Isocrhone(speed=walking_speed)

    def get_isochrone(self,
                      g: nx.MultiDiGraph,
                      orig: NodeId,
                      dest_nodes: Set[NodeId],
                      walking_time_limit: int = 900,
                      bike_time_limit: int = 1800,
                      callback: Callable=lambda *args, **kwargs: None) -> Dict[NodeId, Cost]:

        first_res = self._first_isochrone.get_isochrone(g, orig, self._bss_nodes, limit=walking_time_limit)
        for node, cost in first_res.items():
            self._second_isochrone.init_origin(g, node, cost.secs, cost.secs * BIKE_SPEED)

        print(first_res.get(25207327))
        second_res = self._second_isochrone.run(g, orig, self._bss_nodes - set(first_res.keys()), limit=bike_time_limit)
        print(second_res.get(708253753))

        for node, cost in second_res.items():
            self._third_isochrone.init_origin(g, node, cost.secs, cost.init_cost + (cost.secs - cost.init_secs) * WALKING_SPEED)

        return self._third_isochrone.run(g, orig, dest_nodes, limit=walking_time_limit)
