from dataclasses import dataclass
import networkx as nx
from typing import *

from .inner_types import *
from .isochrone import Isocrhone
from .double_astar import DoubleAstar
from .astar import AStar
from call_backs import double_astar_call_back

WALKING_SPEED = 1.4
BIKE_SPEED = 3.3


@dataclass
class MultiModalAStart(object):

    _foward_isocrhone: Isocrhone = Isocrhone(speed=WALKING_SPEED)
    _backward_isocrhone: Isocrhone = Isocrhone(speed=WALKING_SPEED)

    _double_astar: DoubleAstar = DoubleAstar(speed=BIKE_SPEED)

    def init(self):
        self._foward_isocrhone.init()
        self._backward_isocrhone.init()
        self._double_astar.init()

    def get_best_path(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, bss_nodes: Set[NodeId]) -> \
        Tuple[Tuple[List[NodeId], List[NodeId], List[NodeId]], Tuple[float, float, float]]:

        self.init()
        forward_isochrone = self._foward_isocrhone.get_isochrone(g, orig, bss_nodes, limit=900)
        backward_isochrone = self._backward_isocrhone.get_isochrone(g, dest, bss_nodes, limit=900)

        if not forward_isochrone or not backward_isochrone:
            return []

        for node, cost in forward_isochrone.items():
            self._double_astar.init_forward(g, node, dest, 0, cost.secs * BIKE_SPEED)

        for node, cost in backward_isochrone.items():
            self._double_astar.init_backward(g, orig, node, 0, cost.secs * BIKE_SPEED)

        bss_route, bike_secs = self._double_astar.run(g, orig, dest)

        forward_bss = bss_route[0]
        backward_bss = bss_route[-1]

        forward_walking_route, forward_secs = AStar(speed=WALKING_SPEED).get_best_path(g, orig, forward_bss)

        backward_walking_route, backward_secs = AStar(speed=WALKING_SPEED).get_best_path(g, backward_bss, dest)

        return (forward_walking_route, bss_route, backward_walking_route), (forward_secs, bike_secs, backward_secs)

