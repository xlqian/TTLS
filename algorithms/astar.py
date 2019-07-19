from dataclasses import dataclass
from typing import List, Dict, Callable, Tuple
from collections import defaultdict
import networkx as nx
import weakref
from priority_queue import PriorityQueue

from algorithms.inner_types import NodeId, EdgeId, Cost, EdgeLabel, EdgeStatus, EdgeLabelIdx
from algorithms.utils import osm_to_pointll


@dataclass
class BestPath(object):
    edge_label_index: EdgeLabelIdx
    cost: Cost


@dataclass
class AStar(object):
    _edge_labels: List[EdgeLabel]
    _destinations: Dict[EdgeId, Cost]
    _adjacency_list: PriorityQueue
    _edges_status: Dict[EdgeId, EdgeStatus]
    _orig: NodeId = -1
    _dest: NodeId = - 1
    _cost_factor: float = 0.4
    _best_path: BestPath = BestPath(-1, Cost(0, 0))
    _speed: float = 1.4

    def init(self):
        self._edge_labels = []
        self._destinations = defaultdict(Cost)
        self._adjacency_list = PriorityQueue()
        self._edges_status = defaultdict(EdgeStatus)
        self._best_path = BestPath(-1, Cost(0, 0))

    def __init__(self, speed=1.4, cost_factor=0.4):
        self.init()
        self._speed = speed
        self._cost_factor = cost_factor

    def _get_edge_cost(self, label):
        return self._edge_labels[label]

    def _get_edge_status(self, edge_id: EdgeId) -> EdgeStatus:
        s = self._edges_status.get(edge_id)
        if not s:
            return self._edges_status[edge_id].set_unreached()
        return s

    def _get_heuristic_cost(self, g: nx.MultiDiGraph, start_node: NodeId, end_node: NodeId) -> float:
        if self._cost_factor == 0 or end_node is None:
            return 0
        start_ll = osm_to_pointll(g, start_node)
        end_ll = osm_to_pointll(g, end_node)
        return start_ll.distance_to(end_ll) * self._cost_factor

    def expand_forward(self, g, node, pred_idx, dest):

        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels[pred_idx]

            new_cost = pred.cost + Cost(edge['length'], edge['length'] / self._speed)

            new_cost.init_cost = pred.cost.init_cost
            new_cost.init_secs = pred.cost.init_secs

            edge_id = EdgeId(node, end_node)

            d = self._destinations.get(edge_id)
            if d is not None:
                if self._best_path.edge_label_index is -1 or new_cost < self._best_path.cost:
                    self._best_path.edge_label_index = edge_status.edge_label_index if edge_status.is_temporary() \
                                                                              else len(self._edge_labels)
                    self._best_path.cost = new_cost

            sort_cost = new_cost.cost + self._get_heuristic_cost(g, end_node, self._dest)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels[edge_status.edge_label_index])
                if new_cost < lab.cost:
                    self._adjacency_list.insert(new_key=sort_cost,
                                                item=edge_status.edge_label_index)
                    lab.pred_idx = pred_idx
                    lab.end_node = end_node
                    lab.cost = new_cost
                continue

            idx = len(self._edge_labels)
            self._edge_labels.append(EdgeLabel(new_cost,
                                               sort_cost,
                                               edge_id,
                                               pred_idx,
                                               end_node))

            self._edges_status[edge_id] = EdgeStatus(idx).set_temporary()

            self._adjacency_list.insert(sort_cost, idx)

    def make_osm_path(self):
        res = []

        edge_label_idx = self._best_path.edge_label_index
        edge_label = self._edge_labels[edge_label_idx]
        res.append(edge_label.end_node)

        while not edge_label.is_origin:
            edge_label_idx = edge_label.pred_idx
            edge_label = self._edge_labels[edge_label_idx]
            res.append(edge_label.end_node)

        res.append(edge_label.edge_id.start)

        res = res[::-1]
        return res, self._best_path.cost.secs

    def init_origin(self, g, orig, init_secs=0, init_cost=0):

        # init origin
        for end_node in g.adj[orig]:
            edge = g.adj[orig][end_node][0]

            secs = edge['length'] / self._speed + init_secs
            sort_cost = edge['length'] + self._get_heuristic_cost(g, end_node, self._dest) + init_cost

            idx = len(self._edge_labels)
            self._edge_labels.append(EdgeLabel(Cost(sort_cost, secs, init_cost, init_secs),
                                               sort_cost,
                                               EdgeId(orig, end_node),
                                               -1,
                                               end_node,
                                               True))
            self._adjacency_list.insert(sort_cost, idx)

    def get_best_path(self,
                      g: nx.MultiDiGraph,
                      orig: NodeId,
                      dest: NodeId,
                      callback: Callable=lambda *args, **kwargs: None) -> Tuple[List[NodeId], float]:

        self._orig = orig
        self._dest = dest

        self.init()

        self.init_origin(g, orig)

        # init destination
        for end_node in g.adj[dest]:
            edge = g.adj[dest][end_node][0]
            self._destinations[EdgeId(dest, end_node)] = edge['length']

        i = 0
        a = 0
        # begin search
        while True:
            if i % 200 == 0:
                callback(g, orig, dest, self._edges_status, self._edge_labels, str(a).zfill(4))
                a += 1
            i += 1

            current_labels = len(self._edge_labels)

            if current_labels > PriorityQueue.QUEUE_MAX_SIZE:
                return [], -1.

            _, pred_index = self._adjacency_list.pop()
            pred_edge_label = self._edge_labels[pred_index]

            # Do we touch the destination?
            d = self._destinations.get(pred_edge_label.edge_id)
            if d:
                if self._best_path.edge_label_index is -1:
                    self._best_path.edge_label_index = pred_index
                    self._best_path.cost = pred_edge_label.cost

                return self.make_osm_path()

            if not pred_edge_label.is_origin:
                self._edges_status[pred_edge_label.edge_id].set_permanent()

            self.expand_forward(g, pred_edge_label.end_node, pred_index, dest)
