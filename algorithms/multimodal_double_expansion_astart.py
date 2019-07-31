from dataclasses import dataclass
from typing import *
from collections import defaultdict
import weakref

import networkx as nx

from .inner_types import *
from priority_queue import PriorityQueue
from algorithms import utils
from algorithms.inner_types import NodeId, EdgeId, Cost, EdgeLabel, EdgeStatus, EdgeLabelIdx


kThresholdDelta = 200.

WALKING_SPEED = 1.1
BIKE_SPEED = 3.3


@dataclass
class BestConnection:
    forward: EdgeId = EdgeId(-1, -1)
    backward: EdgeId = EdgeId(-1, -1)
    cost: float = float('inf')
    mode: str = None


@dataclass
class MultiModalDoubleExpansionAStar(object):

    # Walking
    _edge_labels_walking_forward: List[EdgeLabel] = field(default_factory=list)
    _edge_labels_walking_backward: List[EdgeLabel] = field(default_factory=list)

    _adjacency_list_walking_forward: PriorityQueue = PriorityQueue()
    _adjacency_list_walking_backward: PriorityQueue = PriorityQueue()

    _edges_status_walking_forward: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))
    _edges_status_walking_backward: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))

    # Bike
    _edge_labels_bike_forward: List[EdgeLabel] = field(default_factory=list)
    _edge_labels_bike_backward: List[EdgeLabel] = field(default_factory=list)

    _adjacency_list_bike_forward: PriorityQueue = PriorityQueue()
    _adjacency_list_bike_backward: PriorityQueue = PriorityQueue()

    _edges_status_bike_forward: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))
    _edges_status_bike_backward: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))

    _cost_factor: float = 0.2
    _best_path: BestConnection = BestConnection()

    _forward_walking_bss: Dict[NodeId, EdgeLabelIdx] = field(default_factory=dict)
    _backward_walking_bss: Dict[NodeId, EdgeLabelIdx] = field(default_factory=dict)

    _walking_speed: float = WALKING_SPEED
    _bike_speed: float = BIKE_SPEED

    _threshold: float = float('inf')

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
        return self._get_heuristic_cost_impl(g, start_node, end_node, self._cost_factor)

    @staticmethod
    def _get_edge_status_impl(container: Dict[EdgeId, EdgeStatus], edge_id: EdgeId) -> EdgeStatus:
        s = container.get(edge_id)
        if not s:
            return container[edge_id].set_unreached()
        return s

    def _get_edge_status_walking_forward(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status_walking_forward, edge_id)

    def _get_edge_status_walking_backward(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status_walking_backward, edge_id)

    def _get_edge_status_bike_forward(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status_bike_forward, edge_id)

    def _get_edge_status_bike_backward(self, edge_id: EdgeId) -> EdgeStatus:
        return self._get_edge_status_impl(self._edges_status_bike_backward, edge_id)

    def init_forward(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[orig]:
            edge = g.adj[orig][end_node][0]

            secs = edge['length'] / self._walking_speed + init_secs
            cost = edge['length'] + init_cost
            sort_cost = cost + self._get_walking_heuristic_cost(g, end_node, dest)

            idx = len(self._edge_labels_walking_forward)
            self._edge_labels_walking_forward.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                                     sort_cost,
                                                     EdgeId(orig, end_node),
                                                     -1,
                                                     end_node,
                                                     is_origin=True,
                                                     is_destination=False))
            self._adjacency_list_walking_forward.insert(sort_cost, idx)
            self._edges_status_walking_forward[EdgeId(orig, end_node)] = EdgeStatus(idx).set_temporary()

    def append_bike_forward(self, g: nx.MultiDiGraph, bss_node: NodeId, dest: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[bss_node]:
            edge = g.adj[bss_node][end_node][0]

            secs = edge['length'] / self._bike_speed + init_secs
            cost = edge['length'] * (self._walking_speed / self._bike_speed) + init_cost
            sort_cost = cost + self._get_bike_heuristic_cost(g, end_node, dest)

            # let's see if the edge has already been visited?
            edge_status = self._edges_status_bike_forward[EdgeId(bss_node, end_node)]
            if edge_status.is_permanent():
                # if it's permanent is this edge has less cost?
                edge_label_idx = edge_status.edge_label_index
                lab = weakref.proxy(self._edge_labels_bike_forward[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if cost > lab.cost.cost:
                        continue
                    self._adjacency_list_bike_forward.insert(new_key=sort_cost,
                                                             item=edge_label_idx)
                    lab.pred_idx = -1
                    lab.end_node = end_node
                    lab.cost = Cost(cost, secs, init_cost, init_secs)
                    lab.is_origin = True
                    lab.is_destination = False

            else:
                idx = len(self._edge_labels_bike_forward)
                self._edge_labels_bike_forward.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                                         sort_cost,
                                                         EdgeId(bss_node, end_node),
                                                         -1,
                                                         end_node,
                                                         is_origin=True,
                                                         is_destination=False))
                self._adjacency_list_bike_forward.insert(sort_cost, idx)
                self._edges_status_bike_forward[EdgeId(bss_node, end_node)] = EdgeStatus(idx).set_temporary()

    def init_backward(self, g: nx.MultiDiGraph, orig: NodeId, dest: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[dest]:
            edge = g.adj[dest][end_node][0]

            secs = edge['length'] / self._walking_speed + init_secs
            cost = edge['length'] + init_cost
            sort_cost = cost + self._get_walking_heuristic_cost(g, end_node, orig)

            idx = len(self._edge_labels_walking_backward)
            self._edge_labels_walking_backward.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                                      sort_cost,
                                                      EdgeId(dest, end_node),
                                                      -1,
                                                      end_node,
                                                      is_origin=False,
                                                      is_destination=True))
            self._adjacency_list_walking_backward.insert(sort_cost, idx)
            self._edges_status_walking_backward[EdgeId(dest, end_node)] = EdgeStatus(idx).set_temporary()

    def append_bike_backward(self, g: nx.MultiDiGraph, bss_node: NodeId, orig: NodeId, init_secs: float = 0, init_cost: float = 0):
        for end_node in g.adj[bss_node]:
            edge = g.adj[bss_node][end_node][0]

            secs = edge['length'] / self._bike_speed + init_secs
            cost = edge['length'] * (self._walking_speed / self._bike_speed) + init_cost
            sort_cost = cost + self._get_bike_heuristic_cost(g, end_node, orig)
            edge_status = self._edges_status_bike_backward[EdgeId(bss_node, end_node)]

            if edge_status.is_permanent():
                # if it's permanent is this edge has less cost?
                edge_label_idx = edge_status.edge_label_index
                lab = weakref.proxy(self._edge_labels_bike_backward[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if cost > lab.cost.cost:
                        continue
                    self._adjacency_list_bike_backward.insert(new_key=sort_cost,
                                                              item=edge_label_idx)
                    lab.pred_idx = -1
                    lab.end_node = end_node
                    lab.cost = Cost(cost, secs, init_cost, init_secs)
                    lab.is_origin = False
                    lab.is_destination = True
            else:
                idx = len(self._edge_labels_bike_backward)
                self._edge_labels_bike_backward.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                                                 sort_cost,
                                                                 EdgeId(bss_node, end_node),
                                                                 -1,
                                                                 end_node,
                                                                 is_origin=False,
                                                                 is_destination=True))
                self._adjacency_list_bike_backward.insert(sort_cost, idx)
                self._edges_status_bike_backward[EdgeId(bss_node, end_node)] = EdgeStatus(idx).set_temporary()

    def expand_walking_forward(self, g: nx.MultiDiGraph, node: NodeId, pred_idx: EdgeLabelIdx, dest: NodeId):
        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status_walking_forward(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels_walking_forward[pred_idx]

            new_cost = pred.cost + Cost(edge['length'], edge['length'] / self._walking_speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost + self._get_walking_heuristic_cost(g, end_node, dest)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels_walking_forward[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if new_cost < lab.cost:
                        self._adjacency_list_walking_forward.insert(new_key=sort_cost,
                                                            item=edge_status.edge_label_index)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost

                # Hmmm, we are visiting the edge in the opposing direction of last visit
                elif lab.end_node == node:
                    if new_cost.cost < (lab.cost.cost - edge['length']):
                        self._adjacency_list_walking_forward.insert(new_key=sort_cost,
                                                            item=edge_status.edge_label_index)
                        lab.edge_id = EdgeId(node, end_node)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                continue

            idx = len(self._edge_labels_walking_forward)
            self._edge_labels_walking_forward.append(EdgeLabel(new_cost,
                                                               sort_cost,
                                                               edge_id,
                                                               pred_idx,
                                                               end_node))

            self._edges_status_walking_forward[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_walking_forward.insert(sort_cost, idx)

    def expand_walking_backward(self, g, node, pred_idx, origin):

        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status_walking_backward(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels_walking_backward[pred_idx]

            new_cost = pred.cost + Cost(edge['length'], edge['length'] / self._walking_speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost + self._get_walking_heuristic_cost(g, end_node, origin)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels_walking_backward[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if new_cost < lab.cost:
                        self._adjacency_list_walking_backward.insert(new_key=sort_cost,
                                                                     item=edge_status.edge_label_index)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost

                # Hmmm, we are visiting the edge in the opposing direction of last visit
                elif lab.end_node == node:
                    if new_cost.cost < (lab.cost.cost - edge['length']):
                        self._adjacency_list_walking_backward.insert(new_key=sort_cost,
                                                                     item=edge_status.edge_label_index)
                        lab.edge_id = EdgeId(node, end_node)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                continue

            idx = len(self._edge_labels_walking_backward)
            self._edge_labels_walking_backward.append(EdgeLabel(new_cost,
                                                                sort_cost,
                                                                edge_id,
                                                                pred_idx,
                                                                end_node))

            self._edges_status_walking_backward[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_walking_backward.insert(sort_cost, idx)

    def expand_bike_forward(self, g: nx.MultiDiGraph, node: NodeId, pred_idx: EdgeLabelIdx, dest: NodeId):
        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status_bike_forward(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels_bike_forward[pred_idx]

            new_cost = pred.cost + Cost(edge['length'] * self._walking_speed / self._bike_speed,
                                        edge['length'] / self._bike_speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost + self._get_bike_heuristic_cost(g, end_node, dest)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels_bike_forward[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if new_cost < lab.cost:
                        self._adjacency_list_bike_forward.insert(new_key=sort_cost,
                                                            item=edge_status.edge_label_index)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost

                # Hmmm, we are visiting the edge in the opposing direction of last visit
                elif lab.end_node == node:
                    if new_cost.cost < (lab.cost.cost - edge['length'] * (self._walking_speed / self._bike_speed)):
                        self._adjacency_list_bike_forward.insert(new_key=sort_cost,
                                                            item=edge_status.edge_label_index)
                        lab.edge_id = EdgeId(node, end_node)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                continue

            idx = len(self._edge_labels_bike_forward)
            self._edge_labels_bike_forward.append(EdgeLabel(new_cost,
                                                            sort_cost,
                                                            edge_id,
                                                            pred_idx,
                                                            end_node))

            self._edges_status_bike_forward[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_bike_forward.insert(sort_cost, idx)

    def expand_bike_backward(self, g, node, pred_idx, origin):
        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._get_edge_status_bike_backward(EdgeId(node, end_node))

            if edge_status.is_permanent():
                continue

            pred = self._edge_labels_bike_backward[pred_idx]

            new_cost = pred.cost + Cost(edge['length'] * self._walking_speed / self._bike_speed,
                                        edge['length'] / self._bike_speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost + self._get_bike_heuristic_cost(g, end_node, origin)

            # the edge has been visited
            if edge_status.is_temporary():
                lab = weakref.proxy(self._edge_labels_bike_backward[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if new_cost < lab.cost:
                        self._adjacency_list_bike_backward.insert(new_key=sort_cost,
                                                                  item=edge_status.edge_label_index)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost

                # Hmmm, we are visiting the edge in the opposing direction of last visit
                elif lab.end_node == node:
                    if new_cost.cost < (lab.cost.cost - edge['length'] * (self._walking_speed/ self._bike_speed)):
                        self._adjacency_list_bike_backward.insert(new_key=sort_cost,
                                                                  item=edge_status.edge_label_index)
                        lab.edge_id = EdgeId(node, end_node)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                continue

            idx = len(self._edge_labels_bike_backward)
            self._edge_labels_bike_backward.append(EdgeLabel(new_cost,
                                                             sort_cost,
                                                             edge_id,
                                                             pred_idx,
                                                             end_node))

            self._edges_status_bike_backward[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_bike_backward.insert(sort_cost, idx)

    def set_forward_walking_connection(self, pred: EdgeId):

        edge_status_backward = self._edges_status_walking_backward.get(pred)
        edge_label_backward = edge_status_backward.edge_label_index

        edge_status_forward = self._edges_status_walking_forward.get(pred)
        edge_label_forward = edge_status_forward.edge_label_index
        pred_idx_forward = self._edge_labels_walking_forward[edge_label_forward].pred_idx

        c = self._edge_labels_walking_backward[edge_label_backward].cost \
            + self._edge_labels_walking_forward[pred_idx_forward].cost

        if c.cost < self._best_path.cost:
            self._best_path = BestConnection(self._edge_labels_walking_forward[pred_idx_forward].edge_id,
                                             self._edge_labels_walking_backward[edge_label_backward].edge_id,
                                             c.cost,
                                             "walking")

    # backward searching reach on a edge reached by forward searching
    def set_backward_walking_connection(self, pred: EdgeId):

        edge_status_forward = self._edges_status_walking_forward.get(pred)
        edge_label_forward = edge_status_forward.edge_label_index

        edge_status_backward = self._edges_status_walking_backward.get(pred)
        edge_label_backward = edge_status_backward.edge_label_index
        pred_idx_backward = self._edge_labels_walking_backward[edge_label_backward].pred_idx

        c = self._edge_labels_walking_backward[pred_idx_backward].cost \
            + self._edge_labels_walking_forward[edge_label_forward].cost

        if c.cost < self._best_path.cost:
            self._best_path = BestConnection(self._edge_labels_walking_forward[edge_label_forward].edge_id,
                                             self._edge_labels_walking_backward[pred_idx_backward].edge_id,
                                             c.cost,
                                             "walking")

    # forward searching reach on a edge reached by backward searching
    def set_forward_bike_connection(self, pred: EdgeId):

        edge_status_backward = self._edges_status_bike_backward.get(pred)
        edge_label_backward = edge_status_backward.edge_label_index

        edge_status_forward = self._edges_status_bike_forward.get(pred)
        edge_label_forward = edge_status_forward.edge_label_index
        pred_idx_forward = self._edge_labels_bike_forward[edge_label_forward].pred_idx

        c = self._edge_labels_bike_backward[edge_label_backward].cost \
            + self._edge_labels_bike_forward[pred_idx_forward].cost

        if c.cost < self._best_path.cost:
            self._best_path = BestConnection(self._edge_labels_bike_forward[pred_idx_forward].edge_id,
                                             self._edge_labels_bike_backward[edge_label_backward].edge_id,
                                             c.cost,
                                             "bike")

    # backward searching reach on a edge reached by forward searching
    def set_backward_bike_connection(self, pred: EdgeId):

        edge_status_forward = self._edges_status_bike_forward.get(pred)
        edge_label_forward = edge_status_forward.edge_label_index

        edge_status_backward = self._edges_status_bike_backward.get(pred)
        edge_label_backward = edge_status_backward.edge_label_index
        pred_idx_backward = self._edge_labels_bike_backward[edge_label_backward].pred_idx

        c = self._edge_labels_bike_backward[pred_idx_backward].cost \
            + self._edge_labels_bike_forward[edge_label_forward].cost

        if c.cost < self._best_path.cost:
            self._best_path = BestConnection(self._edge_labels_bike_forward[edge_label_forward].edge_id,
                                             self._edge_labels_bike_backward[pred_idx_backward].edge_id,
                                             c.cost,
                                             "bike")

    def make_osm_path(self) -> Tuple[List[NodeId], float]:
        print(self._best_path)

        if self._best_path.mode == "walking":
            res_forward = []
            edge = self._best_path.forward

            edge_label_idx = self._edges_status_walking_forward[edge].edge_label_index
            edge_label = self._edge_labels_walking_forward[edge_label_idx]
            forward_secs = edge_label.cost.secs
            while not edge_label.is_origin:
                res_forward.append(edge_label.end_node)
                edge_label_idx = edge_label.pred_idx
                edge_label = self._edge_labels_walking_forward[edge_label_idx]

            res_forward.append(edge_label.edge_id.end)
            res_forward.append(edge_label.edge_id.start)
            res_forward = res_forward[::-1]

            res_backward = []

            edge = self._best_path.backward
            edge_label_idx = self._edges_status_walking_backward[edge].edge_label_index
            edge_label = self._edge_labels_walking_backward[edge_label_idx]
            backward_secs = edge_label.cost.secs

            while not edge_label.is_destination:
                res_backward.append(edge_label.end_node)
                edge_label_idx = edge_label.pred_idx
                edge_label = self._edge_labels_walking_backward[edge_label_idx]

            res_backward.append(edge_label.edge_id.end)
            res_backward.append(edge_label.edge_id.start)

            if res_forward[-1] == res_backward[0]:
                res_forward.pop(-1)
            return res_forward + res_backward, forward_secs + backward_secs
        else:
            res_forward = []
            edge = self._best_path.forward
            edge_label_idx = self._edges_status_bike_forward[edge].edge_label_index
            edge_label = self._edge_labels_bike_forward[edge_label_idx]
            forward_secs = edge_label.cost.secs
            while not edge_label.is_origin:
                res_forward.append(edge_label.end_node)
                edge_label_idx = edge_label.pred_idx
                edge_label = self._edge_labels_bike_forward[edge_label_idx]

            res_forward.append(edge_label.edge_id.end)
            res_forward.append(edge_label.edge_id.start)
            res_forward = res_forward[::-1]

            res_backward = []

            edge = self._best_path.backward
            edge_label_idx = self._edges_status_bike_backward[edge].edge_label_index
            edge_label = self._edge_labels_bike_backward[edge_label_idx]
            backward_secs = edge_label.cost.secs

            while not edge_label.is_destination:
                res_backward.append(edge_label.end_node)
                edge_label_idx = edge_label.pred_idx
                edge_label = self._edge_labels_bike_backward[edge_label_idx]

            res_backward.append(edge_label.edge_id.end)
            res_backward.append(edge_label.edge_id.start)

            if res_forward[-1] == res_backward[0]:
                res_forward.pop(-1)

            return res_forward + res_backward, forward_secs + backward_secs

    def get_best_path(self,
                      g: nx.MultiDiGraph,
                      orig: NodeId,
                      dest: NodeId,
                      bss_nodes: Set[NodeId],
                      callback: Callable = lambda *args, **kwargs: None) -> Tuple[List[NodeId], float]:

        expand_forward = True
        expand_backward = True

        self.init_forward(g, orig, dest)
        self.init_backward(g, orig, dest)

        a = 0
        i = 0

        # in order to balance the research in both directions
        walking_diff = None
        bike_diff = None

        f_cost, _ = self._adjacency_list_walking_forward.peak()
        b_cost, _ = self._adjacency_list_walking_backward.peak()
        walking_diff = f_cost - b_cost

        forward_cost = None
        backward_cost = None

        bss_reached_forward = False
        bss_reached_backward = False

        while True:
            if a % 50 == 0:
                callback(g, orig, dest, bss_nodes,
                         self._edges_status_walking_forward, self._edge_labels_walking_forward,
                         self._edges_status_walking_backward, self._edge_labels_walking_backward,
                         self._edges_status_bike_forward, self._edge_labels_bike_forward,
                         self._edges_status_bike_backward, self._edge_labels_bike_backward,
                         str(i).zfill(4), prefix="double_expansion")
                i += 1
            a += 1

            if expand_forward:

                bike_cost, bike_forward_edge_label_idx = float('inf'), None

                if all((bss_reached_forward, bss_reached_backward)) and self._adjacency_list_bike_forward:
                    bike_cost, bike_forward_edge_label_idx = self._adjacency_list_bike_forward.peak()
                walking_cost, walking_forward_edge_label_idx = self._adjacency_list_walking_forward.peak()

                if not all((bss_reached_forward, bss_reached_backward)) or walking_cost < bike_cost:
                    expand_walking_forward = True
                    expand_bike_forward = False
                    forward_cost = walking_cost
                    self._adjacency_list_walking_forward.pop()
                    forward_edge_label = self._edge_labels_walking_forward[walking_forward_edge_label_idx]

                    # We don't want the expansion to go forever
                    # ???
                    if forward_edge_label.sort_cost - (walking_diff if walking_diff is not None else 0) > self._threshold:
                        return self.make_osm_path()

                    if self._edges_status_walking_backward[forward_edge_label.edge_id].is_permanent():
                        if self._threshold == float('inf'):
                            self._threshold = forward_edge_label.sort_cost + kThresholdDelta
                        self.set_forward_walking_connection(forward_edge_label.edge_id)

                    if forward_edge_label.end_node in bss_nodes:
                        bss_reached_forward = True
                        self._forward_walking_bss[forward_edge_label.end_node] = walking_forward_edge_label_idx
                        self.append_bike_forward(g,
                                                 forward_edge_label.end_node,
                                                 dest,
                                                 0,
                                                 forward_edge_label.cost.secs * self._walking_speed)
                        if bike_diff is None and all((bss_reached_forward, bss_reached_backward)):
                            f_cost, _ = self._adjacency_list_bike_forward.peak()
                            b_cost, _ = self._adjacency_list_bike_backward.peak()
                            bike_diff = f_cost - b_cost
                else:
                    expand_walking_forward = False
                    expand_bike_forward = True
                    forward_cost = bike_cost
                    self._adjacency_list_bike_forward.pop()
                    forward_edge_label = self._edge_labels_bike_forward[bike_forward_edge_label_idx]

                    # We don't want the expansion to go forever
                    # ???
                    if forward_edge_label.sort_cost - (bike_diff if bike_diff is not None else 0) > self._threshold:
                        return self.make_osm_path()

                    if self._edges_status_bike_backward[forward_edge_label.edge_id].is_permanent():
                        if self._threshold == float('inf'):
                            self._threshold = forward_edge_label.sort_cost + kThresholdDelta
                        self.set_forward_bike_connection(forward_edge_label.edge_id)

            if expand_backward:

                bike_cost, bike_backward_edge_label_idx = float('inf'), None

                if all((bss_reached_forward, bss_reached_backward)) and self._adjacency_list_bike_backward:
                    bike_cost, bike_backward_edge_label_idx = self._adjacency_list_bike_backward.peak()
                walking_cost, walking_backward_edge_label_idx = self._adjacency_list_walking_backward.peak()

                if not all((bss_reached_forward, bss_reached_backward)) or walking_cost < bike_cost:
                    expand_walking_backward = True
                    expand_bike_backward = False

                    backward_cost = walking_cost + walking_diff if walking_diff is not None else 0

                    self._adjacency_list_walking_backward.pop()
                    backward_edge_label = self._edge_labels_walking_backward[walking_backward_edge_label_idx]

                    # We don't want the expansion to go forever
                    # ???
                    if backward_edge_label.sort_cost - (walking_diff if walking_diff is not None else 0) > self._threshold:
                        return self.make_osm_path()

                    if self._edges_status_walking_forward[backward_edge_label.edge_id].is_permanent():
                        if self._threshold == float('inf'):
                            self._threshold = backward_edge_label.sort_cost + kThresholdDelta
                        self.set_backward_walking_connection(backward_edge_label.edge_id)

                    if backward_edge_label.end_node in bss_nodes:
                        bss_reached_backward = True
                        self._backward_walking_bss[backward_edge_label.end_node] = walking_backward_edge_label_idx
                        self.append_bike_backward(g,
                                                  backward_edge_label.end_node,
                                                  orig,
                                                  0,
                                                  backward_edge_label.cost.secs * self._walking_speed)
                        if bike_diff is None and all((bss_reached_forward, bss_reached_backward)):
                            f_cost, _ = self._adjacency_list_bike_forward.peak()
                            b_cost, _ = self._adjacency_list_bike_backward.peak()
                            bike_diff = f_cost - b_cost

                else:
                    expand_walking_backward = False
                    expand_bike_backward = True
                    backward_cost = bike_cost + bike_diff if bike_diff is not None else 0
                    self._adjacency_list_bike_backward.pop()
                    backward_edge_label = self._edge_labels_bike_backward[bike_backward_edge_label_idx]

                    # We don't want the expansion to go forever
                    # ???
                    if backward_edge_label.sort_cost - (bike_diff if bike_diff is not None else 0) > self._threshold:
                        return self.make_osm_path()

                    if self._edges_status_bike_forward[backward_edge_label.edge_id].is_permanent():
                        if self._threshold == float('inf'):
                            self._threshold = backward_edge_label.sort_cost + kThresholdDelta
                        self.set_backward_bike_connection(backward_edge_label.edge_id)

            if forward_cost <= backward_cost:
                expand_forward = True
                expand_backward = False

                if expand_walking_forward:
                    if not forward_edge_label.is_origin:
                        self._edges_status_walking_forward[forward_edge_label.edge_id].set_permanent()
                    self.expand_walking_forward(g, forward_edge_label.end_node, walking_forward_edge_label_idx, dest)
                if expand_bike_forward:
                    if not forward_edge_label.is_origin:
                        self._edges_status_bike_forward[forward_edge_label.edge_id].set_permanent()
                    self.expand_bike_forward(g, forward_edge_label.end_node, bike_forward_edge_label_idx, dest)

            else:
                expand_forward = False
                expand_backward = True

                if expand_walking_backward:
                    if not backward_edge_label.is_destination:
                        self._edges_status_walking_backward[backward_edge_label.edge_id].set_permanent()
                    self.expand_walking_backward(g, backward_edge_label.end_node, walking_backward_edge_label_idx, orig)
                if expand_bike_backward:
                    if not backward_edge_label.is_destination:
                        self._edges_status_bike_backward[backward_edge_label.edge_id].set_permanent()
                    self.expand_bike_backward(g, backward_edge_label.end_node, bike_backward_edge_label_idx, orig)
