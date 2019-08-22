from dataclasses import dataclass
from typing import *
from collections import defaultdict
import weakref


import networkx as nx

from .inner_types import *
from priority_queue import PriorityQueue
from algorithms import utils
from algorithms.inner_types import NodeId, EdgeId, Cost, EdgeLabel, EdgeStatus, EdgeLabelIdx


WALKING_SPEED = 1.4
BIKE_SPEED = 3.3


@dataclass
class BestPath(object):
    edge_label_index: EdgeLabelIdx
    cost: Cost


@dataclass
class MultiModalDoubleExpansionAStarOneQueue(object):

    _edge_labels: List[EdgeLabel] = field(default_factory=list)
    _adjacency_list: PriorityQueue = PriorityQueue()
    _edges_status: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))

    _cost_factor: float = 0

    _walking_speed: float = WALKING_SPEED
    _bike_speed: float = BIKE_SPEED

    _threshold: float = float('inf')

    _destinations: Dict[EdgeId, BestPath] = field(default_factory=dict)

    _best_path: BestPath = BestPath(-1, Cost(0, 0))

    @staticmethod
    def _get_heuristic_cost_impl(g: nx.MultiDiGraph, start_node: NodeId, end_node: NodeId, cost_factor: float) -> float:
        if cost_factor == 0 or end_node is None:
            return 0
        start_ll = utils.osm_to_pointll(g, start_node)
        end_ll = utils.osm_to_pointll(g, end_node)
        return start_ll.distance_to(end_ll) * cost_factor

    def _get_walking_heuristic_cost(self, g: nx.MultiDiGraph, start_node: NodeId, end_node: NodeId) -> float:
        return self._get_heuristic_cost_impl(g, start_node, end_node, self._cost_factor)

    def _get_bike_heuristic_cost(self, g: nx.MultiDiGraph, start_node: NodeId, end_node: NodeId) -> float:
        return self._get_heuristic_cost_impl(g, start_node, end_node, self._cost_factor) * 1.2 * (self._walking_speed / self._bike_speed)

    @staticmethod
    def _get_edge_status_impl(container: Dict[EdgeId, EdgeStatus], edge_id: EdgeId) -> EdgeStatus:
        s = container.get(edge_id)
        if not s:
            return container[edge_id].set_unreached()
        return s

    def _get_edge_status(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status, edge_id)

    def init_origin(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[orig]:
            edge = g.adj[orig][end_node][0]

            secs = edge['length'] / self._walking_speed + init_secs
            cost = edge['length'] + init_cost
            sort_cost = cost + self._get_walking_heuristic_cost(g, end_node, dest)

            idx = len(self._edge_labels)
            self._edge_labels.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                               sort_cost,
                                               EdgeId(orig, end_node),
                                               -1,
                                               end_node,
                                               is_origin=True,
                                               is_destination=False))
            self._adjacency_list.insert(sort_cost, idx)
            self._edges_status[EdgeId(orig, end_node)] = EdgeStatus(idx).set_temporary()

    def init_destination(self,  g: nx.MultiDiGraph, dest: NodeId):
        # init destination
        for end_node in g.adj[dest]:
            edge = g.adj[dest][end_node][0]
            self._destinations[EdgeId(dest, end_node, mode=TravelMode.WALKING)] = edge['length']

    def expand_forward(self, g: nx.MultiDiGraph, node: NodeId, pred_idx: EdgeLabelIdx, dest: NodeId, bss_nodes: Set[NodeId]):

        def _get_speed(travel_mode: TravelMode):
            return (self._walking_speed, self._bike_speed)[travel_mode.value]

        def _normalize_factor(travel_mode: TravelMode):
            return self._walking_speed / (self._walking_speed, self._bike_speed)[travel_mode.value]

        def _get_heurestic_cost(travel_mode: TravelMode, *args, **kwargs):
            fun = (self._get_walking_heuristic_cost, self._get_bike_heuristic_cost)[travel_mode.value]
            return fun(*args, **kwargs)

        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]

            pred = self._edge_labels[pred_idx]

            edge_status = self._get_edge_status(EdgeId(node, end_node, mode=pred.edge_id.mode))

            if node not in bss_nodes and edge_status.is_permanent():
                continue

            if node not in bss_nodes and not edge_status.is_permanent():
                new_cost = pred.cost + Cost(edge['length'] * _normalize_factor(pred.edge_id.mode),
                                            edge['length'] / _get_speed(pred.edge_id.mode))
                edge_id = EdgeId(node, end_node, pred.edge_id.mode)
                sort_cost = new_cost.cost + _get_heurestic_cost(pred.edge_id.mode, g, end_node, dest)
                # the edge has been visited
                if edge_status.is_temporary():
                    lab = weakref.proxy(self._edge_labels[edge_status.edge_label_index])
                    if lab.end_node == end_node:
                        if new_cost < lab.cost:
                            self._adjacency_list.insert(new_key=sort_cost,
                                                        item=edge_status.edge_label_index)
                            lab.edge_id = EdgeId(node, end_node, pred.edge_id.mode)
                            lab.pred_idx = pred_idx
                            lab.end_node = end_node
                            lab.cost = new_cost

                    # Hmmm, we are visiting the edge in the opposing direction of last visit
                    elif lab.end_node == node:
                        if new_cost.cost < (lab.cost.cost - edge['length'] * _normalize_factor(pred.edge_id.mode)):
                            self._adjacency_list.insert(new_key=sort_cost,
                                                        item=edge_status.edge_label_index)
                            lab.edge_id = EdgeId(node, end_node, pred.edge_id.mode)
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

            if node in bss_nodes:
                for mode in TravelMode:
                    edge_status = self._get_edge_status(EdgeId(node, end_node, mode=mode))

                    if edge_status.is_permanent():
                        continue
                    new_cost = pred.cost + Cost(edge['length'] * _normalize_factor(mode),
                                                edge['length'] / _get_speed(mode))

                    edge_id = EdgeId(node, end_node, mode)
                    if mode == TravelMode.WALKING:
                        sort_cost = new_cost.cost + self._get_walking_heuristic_cost(g, end_node, dest)
                    else:
                        sort_cost = new_cost.cost + self._get_bike_heuristic_cost(g, end_node, dest)

                    # the edge has been visited
                    if edge_status.is_temporary():
                        lab = weakref.proxy(self._edge_labels[edge_status.edge_label_index])
                        if lab.end_node == end_node:
                            if new_cost < lab.cost:
                                self._adjacency_list.insert(new_key=sort_cost,
                                                            item=edge_status.edge_label_index)
                                # lab.edge_id = EdgeId(node, end_node, mode)
                                lab.pred_idx = pred_idx
                                lab.end_node = end_node
                                lab.cost = new_cost
                        # Hmmm, we are visiting the edge in the opposing direction of last visit
                        elif lab.end_node == node:
                            if new_cost.cost < (lab.cost.cost - edge['length'] * _normalize_factor(mode)):
                                self._adjacency_list.insert(new_key=sort_cost,
                                                            item=edge_status.edge_label_index)
                                # lab.edge_id = EdgeId(node, end_node, mode)
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

    def make_osm_path(self) -> Tuple[List[List[NodeId]], float]:
        # build path from the last walking

        second_walking = []
        bike = []
        first_walking = []

        edge_label_idx = self._best_path.edge_label_index
        edge_label = self._edge_labels[edge_label_idx]
        second_walking.append(edge_label.end_node)

        changed_mode = 0
        old_mode = edge_label.edge_id.mode

        while not edge_label.is_origin:

            edge_label_idx = edge_label.pred_idx
            edge_label = self._edge_labels[edge_label_idx]

            if old_mode != edge_label.edge_id.mode:
                changed_mode += 1
                old_mode = edge_label.edge_id.mode

            if changed_mode == 0:
                second_walking.append(edge_label.end_node)
            elif changed_mode == 1:
                if not bike:
                    second_walking.append(edge_label.end_node)
                bike.append(edge_label.end_node)
            elif changed_mode == 2:
                if not first_walking:
                    bike.append(edge_label.end_node)
                first_walking.append(edge_label.end_node)

        first_walking.append(edge_label.edge_id.start)

        first_walking = first_walking[::-1]
        bike = bike[::-1]

        second_walking = second_walking[::-1]

        res = []

        if first_walking and bike:
            res.append(first_walking)
            res.append(bike)

        res.append(second_walking)
        return res, self._best_path.cost.secs

    def get_best_path(self,
                      g: nx.MultiDiGraph,
                      orig: NodeId,
                      dest: NodeId,
                      bss_nodes: Set[NodeId],
                      callback: Callable = lambda *args, **kwargs: None) -> Tuple[List[List[NodeId]], float]:

        self.init_origin(g, orig, dest)
        self.init_destination(g, dest)

        i = 0
        a = 0
        # begin search
        while True:
            if i % 10 == 0 and 0:
                callback(g, orig, dest, bss_nodes, self._edges_status, self._edge_labels, str(a).zfill(4))
                a += 1
            i += 1

            current_labels = len(self._edge_labels)

            if current_labels > PriorityQueue.QUEUE_MAX_SIZE:
                return []

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

            self.expand_forward(g, pred_edge_label.end_node, pred_index, dest, bss_nodes)
