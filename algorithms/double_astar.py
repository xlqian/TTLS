from dataclasses import dataclass
from typing import List, Dict, Callable, Tuple
from collections import defaultdict
import networkx as nx
import weakref
from priority_queue import PriorityQueue
from algorithms import utils
from algorithms.inner_types import NodeId, EdgeId, Cost, EdgeLabel, EdgeStatus, EdgeLabelIdx

kThresholdDelta = 20.


@dataclass
class BestConnection:
    forward: EdgeId
    backward: EdgeId
    cost: float


class DoubleAstar(object):
    _edge_labels_forward: List[EdgeLabel] = []
    _edge_labels_backward: List[EdgeLabel] = []

    _adjacency_list_forward: PriorityQueue = PriorityQueue()
    _adjacency_list_backward: PriorityQueue = PriorityQueue()

    _edges_status_forward: Dict[EdgeId, EdgeStatus] = defaultdict(EdgeStatus)
    _edges_status_backward: Dict[EdgeId, EdgeStatus] = defaultdict(EdgeStatus)

    _cost_factor: float = 1
    _best_path: BestConnection = BestConnection(EdgeId(-1, -1),
                                                EdgeId(-1, -1),
                                                float('inf'))
    _speed: float = 1.4
    _threshold: float = float('inf')

    def __init__(self, speed=1.4, cost_factor=1.0):
        self._speed = speed
        self._cost_factor = cost_factor

    def _get_heuristic_cost(self, g: nx.MultiDiGraph, start_node: NodeId, end_node: NodeId) -> float:
        if self._cost_factor == 0 or end_node is None:
            return 0
        start_ll = utils.osm_to_pointll(g, start_node)
        end_ll = utils.osm_to_pointll(g, end_node)

        return start_ll.distance_to(end_ll) * self._cost_factor

    @staticmethod
    def _get_edge_status_impl(container: Dict[EdgeId, EdgeStatus], edge_id: EdgeId):
        s = container.get(edge_id)
        if not s:
            return container[edge_id].set_unreached()
        return s

    def _get_edge_status_forward(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status_forward, edge_id)

    def _get_edge_status_backward(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status_backward, edge_id)

    def init_forward(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[orig]:
            edge = g.adj[orig][end_node][0]

            secs = edge['length'] / self._speed + init_secs
            cost = edge['length']
            sort_cost = cost + self._get_heuristic_cost(g, end_node, dest) + init_cost

            idx = len(self._edge_labels_forward)
            self._edge_labels_forward.append(EdgeLabel(Cost(cost, secs),
                                                       sort_cost,
                                                       EdgeId(orig, end_node),
                                                       -1,
                                                       end_node,
                                                       is_origin=True,
                                                       is_destination=False))
            self._adjacency_list_forward.insert(sort_cost, idx)
            self._edges_status_forward[EdgeId(orig, end_node)] = EdgeStatus(idx).set_temporary()

    def init_backward(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[dest]:
            edge = g.adj[dest][end_node][0]

            secs = edge['length'] / self._speed + init_secs
            cost = edge['length']
            sort_cost = cost + self._get_heuristic_cost(g, end_node, orig) + init_cost

            idx = len(self._edge_labels_backward)
            self._edge_labels_backward.append(EdgeLabel(Cost(cost, secs),
                                                        sort_cost,
                                                        EdgeId(dest, end_node),
                                                        -1,
                                                        end_node,
                                                        is_origin=False,
                                                        is_destination=True))
            self._adjacency_list_backward.insert(sort_cost, idx)
            self._edges_status_backward[EdgeId(dest, end_node)] = EdgeStatus(idx).set_temporary()

    def init(self):
        self._edge_labels_forward = []
        self._edge_labels_backward = []

        self._adjacency_list_forward = PriorityQueue()
        self._adjacency_list_backward = PriorityQueue()

        self._edges_status_forward.clear()
        self._edges_status_backward.clear()

        self._best_path = BestConnection(EdgeId(-1, -1),
                                         EdgeId(-1, -1),
                                         float('inf'))
        self._threshold: float = float('inf')

    # forward searching reach on a edge reached by backward searching
    def set_forward_connection(self, pred: EdgeId):

        edge_status_backward = self._edges_status_backward.get(pred)
        edge_label_backward = edge_status_backward.edge_label_index

        edge_status_forward = self._edges_status_forward.get(pred)
        edge_label_forward = edge_status_forward.edge_label_index
        pred_idx_forward = self._edge_labels_forward[edge_label_forward].pred_idx

        c = self._edge_labels_backward[edge_label_backward].cost \
            + self._edge_labels_forward[pred_idx_forward].cost

        if c.cost < self._best_path.cost:
            self._best_path = BestConnection(self._edge_labels_forward[pred_idx_forward].edge_id,
                                             self._edge_labels_backward[edge_label_backward].edge_id,
                                             c.cost)

    # backward searching reach on a edge reached by forward searching
    def set_backward_connection(self, pred: EdgeId):

        edge_status_forward = self._edges_status_forward.get(pred)
        edge_label_forward = edge_status_forward.edge_label_index

        edge_status_backward = self._edges_status_backward.get(pred)
        edge_label_backward = edge_status_backward.edge_label_index
        pred_idx_backward = self._edge_labels_backward[edge_label_backward].pred_idx

        c = self._edge_labels_backward[pred_idx_backward].cost \
            + self._edge_labels_forward[edge_label_forward].cost

        if c.cost < self._best_path.cost:
            self._best_path = BestConnection(self._edge_labels_forward[edge_label_forward].edge_id,
                                             self._edge_labels_backward[pred_idx_backward].edge_id,
                                             c.cost)

    def expand_forward(self, g: nx.MultiDiGraph, node: NodeId, pred_idx: EdgeLabelIdx, dest: NodeId):

        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status_forward(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels_forward[pred_idx]

            new_cost = pred.cost + Cost(edge['length'], edge['length'] / self._speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost + self._get_heuristic_cost(g, end_node, dest)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels_forward[edge_status.edge_label_index])
                if new_cost < lab.cost:
                    self._adjacency_list_forward.insert(new_key=sort_cost,
                                                        item=edge_status.edge_label_index)
                    lab.pred_idx = pred_idx
                    lab.end_node = end_node
                    lab.cost = new_cost
                continue

            idx = len(self._edge_labels_forward)
            self._edge_labels_forward.append(EdgeLabel(new_cost,
                                                       sort_cost,
                                                       edge_id,
                                                       pred_idx,
                                                       end_node))

            self._edges_status_forward[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_forward.insert(sort_cost, idx)

    def expand_backward(self, g, node, pred_idx, origin):

        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status_backward(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels_backward[pred_idx]

            new_cost = pred.cost + Cost(edge['length'], edge['length'] / self._speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost + self._get_heuristic_cost(g, end_node, origin)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels_backward[edge_status.edge_label_index])
                if new_cost < lab.cost:
                    self._adjacency_list_backward.insert(new_key=sort_cost,
                                                         item=edge_status.edge_label_index)
                    lab.pred_idx = pred_idx
                    lab.end_node = end_node
                    lab.cost = new_cost
                continue

            idx = len(self._edge_labels_backward)
            self._edge_labels_backward.append(EdgeLabel(new_cost,
                                                        sort_cost,
                                                        edge_id,
                                                        pred_idx,
                                                        end_node))

            self._edges_status_backward[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_backward.insert(sort_cost, idx)

    def make_osm_path(self):
        res_forward = []
        edge = self._best_path.forward
        edge_label_idx = self._edges_status_forward[edge].edge_label_index
        edge_label = self._edge_labels_forward[edge_label_idx]
        forward_secs = edge_label.cost.secs
        while not edge_label.is_origin:
            edge_label_idx = edge_label.pred_idx
            edge_label = self._edge_labels_forward[edge_label_idx]
            res_forward.append(edge_label.end_node)

        res_forward.append(edge_label.edge_id.start)
        res_forward = res_forward[::-1]

        res_backward = []

        edge = self._best_path.backward
        edge_label_idx = self._edges_status_backward[edge].edge_label_index
        edge_label = self._edge_labels_backward[edge_label_idx]
        backward_secs = edge_label.cost.secs

        res_backward.append(edge.end)

        while not edge_label.is_destination:
            edge_label_idx = edge_label.pred_idx
            edge_label = self._edge_labels_backward[edge_label_idx]
            res_backward.append(edge_label.end_node)

        res_backward.append(edge_label.edge_id.start)
        res_forward.extend(res_backward)
        return res_forward, forward_secs + backward_secs

    def run(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, callback: Callable = lambda *args, **kwargs: None) -> Tuple[List[NodeId], float]:
        expand_forward = True
        expand_backward = True

        forward_edge_label = None
        forward_edge_label_idx = None
        backward_edge_label = None
        backward_edge_label_idx = None

        a = 0
        i = 0
        while True:

            if a % 15 == 0:
                callback(g, orig, dest,
                         self._edges_status_forward, self._edge_labels_forward,
                         self._edges_status_backward, self._edge_labels_backward,
                         str(i).zfill(4))
                i += 1
            a += 1

            if expand_forward:
                _, forward_edge_label_idx = self._adjacency_list_forward.pop()
                forward_edge_label = self._edge_labels_forward[forward_edge_label_idx]

                # We don't want the expansion to go forever
                if forward_edge_label.sort_cost > self._threshold:
                    return self.make_osm_path()

                if self._edges_status_backward[forward_edge_label.edge_id].is_permanent():
                    if self._threshold == float('inf'):
                        self._threshold = forward_edge_label.sort_cost + kThresholdDelta

                    self.set_forward_connection(forward_edge_label.edge_id)

            if expand_backward:
                _, backward_edge_label_idx = self._adjacency_list_backward.pop()
                backward_edge_label = self._edge_labels_backward[backward_edge_label_idx]

                # We don't want the expansion to go forever
                if backward_edge_label.sort_cost > self._threshold:
                    return self.make_osm_path()

                if self._edges_status_forward[backward_edge_label.edge_id].is_permanent():
                    if self._threshold == float('inf'):
                        self._threshold = forward_edge_label.sort_cost + kThresholdDelta

                    self.set_backward_connection(backward_edge_label.edge_id)

            if forward_edge_label.sort_cost < backward_edge_label.sort_cost:
                expand_forward = True
                expand_backward = False

                self._edges_status_forward[forward_edge_label.edge_id].set_permanent()
                self.expand_forward(g, forward_edge_label.end_node, forward_edge_label_idx, dest)

            else:
                expand_forward = False
                expand_backward = True

                self._edges_status_backward[backward_edge_label.edge_id].set_permanent()
                self.expand_backward(g, backward_edge_label.end_node, backward_edge_label_idx, orig)

    def get_best_path(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId,
                      callback: Callable = lambda *args, **kwargs: None) -> Tuple[List[NodeId], float]:
        self.init()

        self.init_forward(g, orig, dest)
        self.init_backward(g, orig, dest)

        return self.run(g, orig, dest, callback)
