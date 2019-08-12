from dataclasses import dataclass
from typing import *
from collections import defaultdict
import weakref

import networkx as nx

from .inner_types import *
from priority_queue import PriorityQueue
from algorithms import utils
from algorithms.inner_types import NodeId, EdgeId, Cost, EdgeLabel, EdgeStatus, EdgeLabelIdx


kThresholdDelta = 50.

WALKING_SPEED = 1.4
BIKE_SPEED = 3.3


@dataclass
class MultiModalDoubleExpansionIsochrone(object):

    # Walking
    _edge_labels_walking: List[EdgeLabel] = field(default_factory=list)
    _adjacency_list_walking: PriorityQueue = PriorityQueue()
    _edges_status_walking: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))

    # Bike
    _edge_labels_bike: List[EdgeLabel] = field(default_factory=list)
    _adjacency_list_bike: PriorityQueue = PriorityQueue()
    _edges_status_bike: Dict[EdgeId, EdgeStatus] = field(default_factory=lambda: defaultdict(EdgeStatus))

    _walking_bss: Dict[NodeId, EdgeLabelIdx] = field(default_factory=dict)
    _backward_walking_bss: Dict[NodeId, EdgeLabelIdx] = field(default_factory=dict)

    _walking_speed: float = WALKING_SPEED
    _bike_speed: float = BIKE_SPEED

    _threshold: float = float('inf')

    def append_walking(self, g, orig, can_change_mode: bool, init_secs: float=0, init_cost: float=0):
        # init origin
        for end_node in g.adj[orig]:
            edge = g.adj[orig][end_node][0]

            secs = edge['length'] / self._walking_speed + init_secs
            cost = edge['length'] + init_cost
            sort_cost = cost

            # let's see if the edge has already been visited?
            edge_status = self._edges_status_walking[EdgeId(orig, end_node, can_change_mode)]
            if not edge_status.is_unreached():
                # if it's visted, is this edge had less cost?
                edge_label_idx = edge_status.edge_label_index
                lab = weakref.proxy(self._edge_labels_walking[edge_label_idx])
                if lab.end_node == end_node:
                    if cost > lab.cost.cost:
                        continue
                    self._adjacency_list_walking.insert(new_key=cost,
                                                        item=edge_label_idx)
                    lab.pred_idx = -1
                    lab.end_node = end_node
                    lab.cost = Cost(cost, secs, init_cost, init_secs)
                    lab.is_origin = True
                    lab.is_destination = False

            else:
                idx = len(self._edge_labels_walking)
                self._edge_labels_walking.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                                           sort_cost,
                                                           EdgeId(orig, end_node),
                                                           -1,
                                                           end_node,
                                                           is_origin=True,
                                                           can_change_mode=can_change_mode))
                self._adjacency_list_walking.insert(sort_cost, idx)
                self._edges_status_walking[EdgeId(orig, end_node, can_change_mode)] = EdgeStatus(idx).set_temporary()

    def append_bike(self, g: nx.MultiDiGraph, bss_node: NodeId, init_secs: float=0, init_cost: float=0):
        for end_node in g.adj[bss_node]:
            edge = g.adj[bss_node][end_node][0]

            secs = edge['length'] / self._bike_speed + init_secs
            cost = edge['length'] * (self._walking_speed / self._bike_speed) + init_cost

            # let's see if the edge has already been visited?
            edge_status = self._edges_status_bike[EdgeId(bss_node, end_node)]
            if not edge_status.is_unreached():
                # if it's visted, is this edge had less cost?
                edge_label_idx = edge_status.edge_label_index
                lab = weakref.proxy(self._edge_labels_bike[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if cost > lab.cost.cost:
                        continue
                    self._adjacency_list_bike.insert(new_key=cost,
                                                     item=edge_label_idx)
                    lab.pred_idx = -1
                    lab.end_node = end_node
                    lab.cost = Cost(cost, secs, init_cost, init_secs)
                    lab.is_origin = True
                    lab.is_destination = False

            else:
                idx = len(self._edge_labels_bike)
                self._edge_labels_bike.append(EdgeLabel(Cost(cost, secs, init_cost, init_secs),
                                                        cost,
                                                        EdgeId(bss_node, end_node),
                                                        -1,
                                                        end_node,
                                                        is_origin=True))
                self._adjacency_list_bike.insert(cost, idx)
                self._edges_status_bike[EdgeId(bss_node, end_node)] = EdgeStatus(idx).set_temporary()

    def expand_walking(self, g: nx.MultiDiGraph, node: NodeId, pred_idx: EdgeLabelIdx):
        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]

            pred = self._edge_labels_walking[pred_idx]

            edge_id = EdgeId(node, end_node, pred.can_change_mode)

            edge_status = self._edges_status_walking[edge_id]

            new_cost = pred.cost + Cost(edge['length'], edge['length'] / self._walking_speed)

            # the edge has been visited
            if not edge_status.is_unreached():
                lab = weakref.proxy(self._edge_labels_walking[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if new_cost < lab.cost:
                        self._adjacency_list_walking.insert(new_key=new_cost.cost,
                                                            item=edge_status.edge_label_index)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                        lab.can_change_mode = pred.can_change_mode

                # Hmmm, we are visiting the edge in the opposing direction of last visit
                elif lab.end_node == node:
                    if new_cost.cost < (lab.cost.cost - edge['length']):
                        self._adjacency_list_walking.insert(new_key=new_cost.cost,
                                                            item=edge_status.edge_label_index)
                        lab.edge_id = EdgeId(node, end_node)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                        lab.can_change_mode = pred.can_change_mode

                continue

            idx = len(self._edge_labels_walking)
            self._edge_labels_walking.append(EdgeLabel(new_cost,
                                                       new_cost.cost,
                                                       edge_id,
                                                       pred_idx,
                                                       end_node,
                                                       can_change_mode=pred.can_change_mode))

            self._edges_status_walking[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_walking.insert(new_cost.cost, idx)

    def expand_bike(self, g: nx.MultiDiGraph, node: NodeId, pred_idx: EdgeLabelIdx):
        for end_node in g.adj[node]:
            edge = g.adj[node][end_node][0]
            edge_status = self._edges_status_bike[EdgeId(node, end_node)]

            pred = self._edge_labels_bike[pred_idx]

            new_cost = pred.cost + Cost(edge['length'] * self._walking_speed / self._bike_speed,
                                        edge['length'] / self._bike_speed)
            edge_id = EdgeId(node, end_node)

            sort_cost = new_cost.cost

            # the edge has been visited
            if not edge_status.is_unreached():
                lab = weakref.proxy(self._edge_labels_bike[edge_status.edge_label_index])
                if lab.end_node == end_node:
                    if new_cost < lab.cost:
                        self._adjacency_list_bike.insert(new_key=sort_cost,
                                                         item=edge_status.edge_label_index)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost

                # Hmmm, we are visiting the edge in the opposing direction of last visit
                elif lab.end_node == node:
                    if new_cost.cost < (lab.cost.cost - edge['length'] * (self._walking_speed / self._bike_speed)):
                        self._adjacency_list_bike.insert(new_key=sort_cost,
                                                         item=edge_status.edge_label_index)
                        lab.edge_id = EdgeId(node, end_node)
                        lab.pred_idx = pred_idx
                        lab.end_node = end_node
                        lab.cost = new_cost
                continue

            idx = len(self._edge_labels_bike)
            self._edge_labels_bike.append(EdgeLabel(new_cost,
                                                            sort_cost,
                                                            edge_id,
                                                            pred_idx,
                                                            end_node))

            self._edges_status_bike[edge_id] = EdgeStatus(idx).set_temporary()
            self._adjacency_list_bike.insert(sort_cost, idx)

    def get_isochrone(self,
                      g: nx.MultiDiGraph,
                      orig: NodeId,
                      dest_nodes: Set[NodeId],
                      bss_nodes: Set[NodeId],
                      callback: Callable=lambda *args, **kwargs: None) -> Dict[NodeId, Cost]:

        self.append_walking(g, orig, can_change_mode=True)

        expand_walking = True
        expand_bike = True

        a = 0
        i = 0
        walking_cost = 0
        while True:

            if a % 100 == 0:
                callback(g, orig, dest_nodes, bss_nodes,
                         self._edges_status_walking, self._edge_labels_walking,
                         self._edges_status_bike, self._edge_labels_bike,
                         str(i).zfill(4))
                i += 1
                a = 0
            a += 1

            bike_cost, bike_edge_label_idx = float('inf'), None
            if self._adjacency_list_bike:
                bike_cost, bike_edge_label_idx = self._adjacency_list_bike.peak()
            walking_cost, walking_edge_label_idx = self._adjacency_list_walking.peak()

            if walking_cost < bike_cost:
                expand_walking = True
                expand_bike = False
                self._adjacency_list_walking.pop()
                forward_edge_label = self._edge_labels_walking[walking_edge_label_idx]

                if forward_edge_label.can_change_mode and forward_edge_label.cost.secs > 1200:
                    continue

                if not forward_edge_label.can_change_mode and (forward_edge_label.cost.secs - forward_edge_label.cost.init_secs)> 1200:
                    continue

                if forward_edge_label.end_node in dest_nodes:
                    print(forward_edge_label.cost.secs)

                if forward_edge_label.can_change_mode and forward_edge_label.end_node in bss_nodes:
                    self.append_bike(g,
                                     forward_edge_label.end_node,
                                     init_secs=forward_edge_label.cost.secs,
                                     init_cost=forward_edge_label.cost.secs * self._walking_speed)

            else:
                expand_walking = False
                expand_bike = True
                self._adjacency_list_bike.pop()
                forward_edge_label = self._edge_labels_bike[bike_edge_label_idx]

                if (forward_edge_label.cost.secs - forward_edge_label.cost.init_secs) > 1800:
                    continue

                if forward_edge_label.end_node in bss_nodes:
                    self.append_walking(g,
                                        forward_edge_label.end_node,
                                        can_change_mode=False,
                                        init_secs=forward_edge_label.cost.secs,
                                        init_cost=forward_edge_label.cost.secs * self._bike_speed)

            if walking_cost > 3600 * self._walking_speed:
                return {}

            if expand_walking:
                self.expand_walking(g, forward_edge_label.end_node, walking_edge_label_idx)
            elif expand_bike:
                self.expand_bike(g, forward_edge_label.end_node, bike_edge_label_idx)
