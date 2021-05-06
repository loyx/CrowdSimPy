import collections
import functools
from math import sqrt
from typing import *
import random


Point = collections.namedtuple("Point", "longitude latitude")


def EuclideanDistance(p1: Point, p2: Point) -> float:
    if p1 == p2:
        return 0
    return sqrt((p1.longitude - p2.longitude) ** 2 + (p1.latitude - p2.latitude) ** 2)


def ManhattanDistance(p1: Point, p2: Point) -> float:
    if p1 == p2:
        return 0
    return abs(p1.longitude - p2.longitude) + abs(p1.latitude - p2.latitude)


class Area:

    def __init__(self, start_point: Point, end_point: Point):
        if any(x == y for x, y in zip(start_point, end_point)):
            raise ValueError("wrong start and end point")

        # 规范感知区域起止点
        self.startPoint = Point(min(start_point[0], end_point[0]), min(start_point[1], end_point[1]))
        self.endPoint = Point(max(start_point[0], end_point[0]), max(start_point[1], end_point[1]))
        self.len = tuple(y - x for x, y in zip(self.startPoint, self.endPoint))
        self.area = self.len[0] * self.len[1]
        self.center = Point(start_point[0] + self.len[0] / 2, start_point[1] + self.len[1] / 2)

    def inRange(self, point: Point):
        return self.startPoint[0] <= point[0] < self.endPoint[0] \
               and self.startPoint[1] <= point[1] < self.endPoint[1]

    def randomLoc(self) -> Point:
        p1 = random.uniform(self.startPoint[0], self.endPoint[0])
        p2 = random.uniform(self.startPoint[1], self.endPoint[1])
        return Point(p1, p2)

    def __contains__(self, item):
        return isinstance(item, Point) and self.inRange(item)

    def __repr__(self):
        return "Area(start:<{0[0]},{0[0]}>, end:<{1[0]},{1[1]}>)".format(self.startPoint, self.endPoint)


class Region(Area):

    def __init__(self, rid, start_point: Point, end_point: Point):
        super().__init__(start_point, end_point)
        assert self.len[0] == self.len[1]
        self.id = rid

        # for plt
        self.represent_loc = self.randomLoc()

    def __repr__(self):
        return "Region(id:{0}, center:<{1[0]},{1[1]}>, size:{2})".format(
            self.id,
            self.center,
            self.size
        )

    @property
    def size(self):
        return self.len[0]

    @functools.lru_cache()
    def dist(self, reg: 'Region'):
        return ManhattanDistance(self.center, reg.center)


class SenseArea(Area):

    def __init__(self, start_point, end_point, unit='px'):
        super().__init__(start_point, end_point)
        self.unit = unit

    def __repr__(self):
        return "SenseArea(start:{0[0]}{1},end:{0[1]}{1})".format(
            self.len,
            self.unit
        )

    def grid(self, granularity: int) -> List[Region]:
        """
        网格化感知区域
        :param granularity: 网格化粒度
        :return: Region
        """
        if any(x % granularity for x in self.len):
            raise ValueError("granularity should be common factor of length")
        cnt = -1
        return [
            Region(cnt := cnt + 1,
                   Point(i*granularity, j*granularity),
                   Point((i+1)*granularity, (j+1)*granularity))
            for i in range(self.len[0] // granularity)
            for j in range(self.len[1] // granularity)
        ]


if __name__ == '__main__':
    sa = SenseArea((0, 0), (100, 100))
    print(sa)
    regs = sa.grid(10)
    print(regs)
