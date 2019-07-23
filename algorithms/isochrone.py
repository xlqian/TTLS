from typing import Set, Callable, Dict

import networkx as nx

from algorithms.astar import AStar
from algorithms.inner_types import NodeId, EdgeLabel, Cost, EdgeId
from priority_queue import PriorityQueue


class Isocrhone(AStar):

    def __init__(self, speed=1.4):
        super().__init__(speed=speed, cost_factor=0)

    def get_isochrone(self, g: nx.MultiDiGraph, orig: NodeId, dest_nodes: Set[NodeId], limit: int=900,
                      callback: Callable=lambda *args, **kwargs: None) -> Dict[NodeId, Cost]:
        self.init_origin(g, orig)
        return self.run(g, orig, dest_nodes, limit, callback=callback)

    def run(self, g: nx.MultiDiGraph, orig: NodeId, dest_nodes: Set[NodeId], limit: int=900,
            callback: Callable=lambda *args, **kwargs: None) -> Dict[NodeId, Cost]:
        self._orig = orig
        self._dest = None

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
                return res

            if len(self._adjacency_list) == 0:
                return res

            _, pred_index = self._adjacency_list.pop()
            pred_edge_label = self._edge_labels[pred_index]

            if pred_edge_label.cost.secs - pred_edge_label.cost.init_secs > limit:
                continue

            # Do we touch the destination?
            if pred_edge_label.edge_id.end in dest_nodes:
                r = res.get(pred_edge_label.edge_id.end)
                if r is not None:
                    res[pred_edge_label.edge_id.end] = min(r, pred_edge_label.cost)
                else:
                    res[pred_edge_label.edge_id.end] = pred_edge_label.cost

            if not pred_edge_label.is_origin:
                self._edges_status[pred_edge_label.edge_id].set_permanent()

            self.expand_forward(g, pred_edge_label.end_node, pred_index, None)
