from typing import Set, Callable

import networkx as nx

from algorithms.astar import AStar
from algorithms.inner_types import NodeId, EdgeLabel, Cost, EdgeId
from priority_queue import PriorityQueue


class Isocrhone(AStar):

    def __init__(self, speed=1.4, cost_factor=0):
        super().__init__(speed, cost_factor)

    def get_isochrone(self, g: nx.MultiDiGraph, orig: NodeId, dest_nodes: Set[NodeId], limit=900,
                      callback: Callable=lambda *args, **kwargs: None):
        self._orig = orig
        self._dest = None
        self._cost_factor = 0

        # init origin
        for end_node in g.adj[orig]:
            edge = g.adj[orig][end_node][0]

            secs = edge['length'] / self._speed
            sort_cost = edge['length'] + self._get_heuristic_cost(g, end_node, self._dest)

            idx = len(self._edge_labels)
            self._edge_labels.append(EdgeLabel(Cost(sort_cost, secs),
                                               sort_cost,
                                               EdgeId(orig, end_node),
                                               -1,
                                               end_node,
                                               True))
            self._adjacency_list.insert(sort_cost, idx)

        res = {}
        i = 0
        a = 0
        # begin search
        while True:
            if i % 200 == 0:
                callback(g, orig, dest_nodes, self._edges_status, self._edge_labels, str(a).zfill(4))
                a += 1
            i += 1

            current_labels = len(self._edge_labels)

            if current_labels > PriorityQueue.QUEUE_MAX_SIZE:
                return []

            if len(self._adjacency_list) == 0:
                return res

            _, pred_index = self._adjacency_list.pop()
            pred_edge_label = self._edge_labels[pred_index]

            # Do we touch the destination?
            if pred_edge_label.cost.secs > limit:
                continue

            if pred_edge_label.edge_id.start in dest_nodes:
                res[pred_edge_label.edge_id.start] = pred_edge_label.cost.secs
            elif pred_edge_label.edge_id.end in dest_nodes:
                res[pred_edge_label.edge_id.end] = pred_edge_label.cost.secs

            if not pred_edge_label.is_origin:
                self._edges_status[pred_edge_label.edge_id].set_permanent()

            self.expand_forward(g, pred_edge_label.end_node, pred_index, None)
