import numbers
from abc import ABC
from typing import List, Dict

from senseArea import Area, Region
from sensor import Sensor


class TimeBase(ABC):

    def __init__(self, s, e):
        self.s = s
        self.e = e

    def __iter__(self):
        return (i for i in (self.s, self.e))

    @property
    def len(self):
        return self.e - self.s

    def __contains__(self, item):
        if not isinstance(item, numbers.Real):
            return False
        return self.s <= item < self.e


class TimeSlot(TimeBase):

    def __init__(self, tid, s, e):
        super().__init__(s, e)
        self.id = tid

    def __repr__(self):
        return f"TimeSlot({self.id}, [{self.s}, {self.e}))"

    def dist(self, ts: 'TimeSlot'):
        return abs(self.id - ts.id)


class TimeRange(TimeBase):

    def __repr__(self):
        return f"TimeRange([{self.s}, {self.e}))"

    def discretize(self, time_granularity: int) -> List[TimeSlot]:
        if self.len % time_granularity:
            raise ValueError("granularity should be factor of length")
        tid = -1
        return [
            TimeSlot(tid := tid + 1, i * time_granularity, (i + 1) * time_granularity)
            for i in range(self.len // time_granularity)
        ]


class Task:

    def __init__(self, tid, r_sensor: Sensor, t_area: Area, time_range: TimeRange):
        self.id = tid
        self.__required_sensor = r_sensor
        self.area: Area = t_area
        self.timeRange: TimeRange = time_range

        self.TR: List[Region] = []
        self.finished_reg: Dict[int, bool] = {}
        self.isFinished = False

    def __repr__(self):
        return "Task({}, {}, {}, {})".format(
            self.id,
            self.__required_sensor,
            self.area,
            self.timeRange
        )

    def adequateSensor(self, sensor: Sensor):
        return sensor.category == self.__required_sensor.category  \
               and sensor.accuracy >= self.__required_sensor.accuracy


if __name__ == '__main__':
    tr = TimeRange(0, 100)
    print(tr)
    print(tr.discretize(5))
