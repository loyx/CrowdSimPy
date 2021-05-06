from abc import ABC
from typing import List, Dict

from senseArea import Area, Region
from sensor import Sensor


class TimeBase(ABC):

    def __init__(self, s, e):
        self.s = s
        self.e = e
        self.len = self.e - self.s

    def __iter__(self):
        return (i for i in (self.s, self.e))


class TimeSlot(TimeBase):

    def __init__(self, tid, s, e, cycle_len):
        super().__init__(s, e)
        self.id = tid
        self.cycle_length = cycle_len

    def __contains__(self, item):
        # 因为模拟的是某一周期性时间段（如：一天中24h），因此在判断某一个时间点是否在该时间段内时，需要对其取模。
        # 我们不对时间点做过多约束，时间点为从时间零点到该事件发生时的秒数(或其他单位)，时间点可以无限大。
        return self.s <= item % self.cycle_length < self.e

    def __repr__(self):
        return f"TimeSlot({self.id}, [{self.s}, {self.e}))"

    def dist(self, ts: 'TimeSlot'):
        return abs(self.id - ts.id)


class TimeCycle(TimeBase):

    def __init__(self, cycle_len):
        super().__init__(0, cycle_len)
        self.cycle_length = cycle_len

    def __repr__(self):
        return f"TimeCycle({self.cycle_length})"

    def discretize(self, time_granularity: int) -> List[TimeSlot]:
        if self.len % time_granularity:
            raise ValueError("granularity should be factor of length")
        tid = -1
        return [
            TimeSlot(tid := tid + 1, i * time_granularity, (i + 1) * time_granularity, self.cycle_length)
            for i in range(self.len // time_granularity)
        ]


class TimeRange(TimeBase):

    def __repr__(self):
        return f"TimeRange([{self.s}, {self.e}))"

    def __contains__(self, item):
        return self.s <= item < self.e


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
        return "Task(id:{}, sensor{}, {}, {})".format(
            self.id,
            self.__required_sensor.id,
            self.TR,
            self.timeRange
        )

    def adequateSensor(self, sensor: Sensor):
        return sensor.category == self.__required_sensor.category  \
               and sensor.accuracy >= self.__required_sensor.accuracy
