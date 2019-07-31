from dataclasses import dataclass, field
from typing import List, Dict
import bisect
import numpy as np


@dataclass(order=True)
class _Element(object):
    key: float = field(compare=True)
    item: object = field(compare=False)


@dataclass
class PriorityQueue(object):
    QUEUE_MAX_SIZE = 50000

    _data: List = field(default_factory=lambda: np.array([_Element(np.iinfo(np.int32).max, None)] * PriorityQueue.QUEUE_MAX_SIZE, dtype=object))
    _items_key: Dict = field(default_factory=dict)
    _data_size: int = 0

    def pop(self):
        # To be optimized with _start and _end
        if self._data_size > 0:
            element = self._data[0]
            self._data[0:self._data_size-1] = self._data[1:self._data_size]
            del self._items_key[element.item]
            self._data_size -= 1
            return element.key, element.item
        return None, None

    def peak(self):
        # To be optimized with _start and _end
        if self._data_size > 0:
            element = self._data[0]
            return element.key, element.item
        return None, None

    def insert(self, new_key, item):
        # find the old item
        old_key = self._items_key.get(item)

        if old_key is not None:
            pos_left = bisect.bisect_left(self._data[0:self._data_size], _Element(old_key, None))
            pos = None
            while self._data[pos_left].key == old_key:
                if self._data[pos_left].item == item:
                    pos = pos_left
                    break
                pos_left += 1

            if self._data[pos].key == new_key:
                return
            right = bisect.bisect_right(self._data[0:self._data_size], _Element(new_key, None))

            if right < pos:
                self._data[right+1:pos+1] = self._data[right:pos]
                self._data[right] = _Element(new_key, item)

            elif right == pos or (right == pos+1):
                self._data[pos] = _Element(new_key, item)

            elif right > pos:
                stop = right if right < len(self._data) else right - 1
                self._data[pos:stop] = self._data[pos+1:stop+1]
                self._data[right-1] = _Element(new_key, item)

            self._items_key[item] = new_key
        else:
            right = bisect.bisect_right(self._data[:self._data_size], _Element(new_key, None))
            self._data[right+1:self._data_size+1] = self._data[right:self._data_size]
            self._data[right] = _Element(new_key, item)
            self._items_key[item] = new_key
            self._data_size += 1

    def __repr__(self):
        return str(self._data[0:self._data_size])

    def __len__(self):
        return self._data_size

    def __getitem__(self, row):
        if isinstance(row, slice):
            if row.start >= self._data_size or row.stop >= self._data_size:
                raise IndexError('Slice is bigger than the data size: {}'.format(self._data_size))
        return self._data[row]

    def __bool__(self):
        return bool(len(self))


if __name__ == '__main__':
    q = PriorityQueue()
    r = 10
    from random import randrange

    for x in range(r):
        q.insert(randrange(0, 1000), x)

    for x in range(r):
        q.insert(x+randrange(-1000, 1000), x)

    for x in range(r):
        q.pop()
