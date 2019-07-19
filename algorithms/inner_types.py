from enum import Enum

from dataclasses import dataclass, field
from typing import Tuple

NodeId = int
EdgeLabelIdx = int


@dataclass(frozen=True)
class EdgeId(object):
    """
    Can be a dict key

    the order of start, node doesn't matter: EdgeId(1, 2) == EdgeId(2, 1)
    """
    start: NodeId = field(hash=False, compare=False)
    end: NodeId = field(hash=False, compare=False)
    key: Tuple[NodeId, NodeId] = field(hash=True, compare=True)

    def __init__(self, start, end):
        object.__setattr__(self, 'start', start)
        object.__setattr__(self, 'end', end)
        object.__setattr__(self, 'key', (min(start, end), max(start, end)))

@dataclass(order=True)
class Cost(object):
    cost: float = field(compare=True)
    secs: float = field(compare=False)

    # used in multimodal ischrone
    init_cost: float = field(compare=False, default=0)
    # used in multimodal ischrone
    init_secs: float = field(compare=False, default=0)

    def __add__(self, other):
        return Cost(self.cost + other.cost, self.secs + other.secs)


@dataclass
class EdgeLabel(object):
    cost: Cost
    sort_cost: float
    edge_id: EdgeId
    pred_idx: EdgeLabelIdx
    end_node: NodeId
    is_origin: bool = False
    # for double astar
    is_destination: bool = False

    def __eq__(self, other):
        return self.edge_id == other.edge_id and self.pred_idx == other.pred_idx


@dataclass
class EdgeStatus(object):

    edge_label_index: EdgeLabelIdx = -1

    class _Status(Enum):
        Unreached = 0
        Permanent = 1
        Temporary = 2

    _status: _Status = _Status.Unreached

    def is_unreached(self) -> bool:
        return self._status == EdgeStatus._Status.Unreached

    def is_temporary(self) -> bool:
        return self._status == EdgeStatus._Status.Temporary

    def is_permanent(self) -> bool:
        return self._status == EdgeStatus._Status.Permanent

    def set_unreached(self) -> 'EdgeStatus':
        self._status = EdgeStatus._Status.Unreached
        return self

    def set_temporary(self) -> 'EdgeStatus':
        self._status = EdgeStatus._Status.Temporary
        return self

    def set_permanent(self) -> 'EdgeStatus':
        self._status = EdgeStatus._Status.Permanent
        return self


@dataclass
class PointLL(object):

    lon: float = 0
    lat: float = 0

    def distance_to(self, other):
        import math
        N_DEG_TO_RAD = 0.01745329238
        EARTH_RADIUS_IN_METERS = 6372797.560856
        lon_arc = (self.lon - other.lon) * N_DEG_TO_RAD
        lon_h = math.sin(lon_arc * 0.5)
        lon_h *= lon_h
        lat_arc = (self.lat - other.lat) * N_DEG_TO_RAD
        lat_h = math.sin(lat_arc * 0.5)
        lat_h *= lat_h
        tmp = math.cos(self.lat * N_DEG_TO_RAD) * math.cos(other.lat * N_DEG_TO_RAD)
        return EARTH_RADIUS_IN_METERS * 2.0 * math.asin(math.sqrt(lat_h + tmp * lon_h))
