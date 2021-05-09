import functools
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

    @functools.lru_cache(None)
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
        self.subtask_status: Dict[int, int] = {}
        self.Finished = False
        self.alive = True

    def __repr__(self):
        return "Task(id:{}, finished:{}, sensor{}, {}, {})".format(
            self.id,
            self.Finished,
            self.__required_sensor.id,
            ["task"+str(t.id) for t in self.TR],
            self.timeRange
        )

    def beginSubTaskTransaction(self, reg: Region, time):
        """
        robot执行感知任务，从开始到结束是一个事物。开始sensing时调用beginSubTaskTransaction，
        结束时调用commitSubTaskTransaction.
        :param reg: 子任务区域
        :param time: 时间
        """
        if time not in self.timeRange:
            raise RuntimeError(f"error begin time {time} for Task{self.id}-subtask:reg{reg.id}")
        self.subtask_status[reg.id] -= 1
        if self.subtask_status[reg.id] < 0:
            raise RuntimeError(f"sensing a finished task{self.id}!")
        if not any(self.subtask_status.values()):
            self.Finished = True
        if time > self.timeRange.e:
            self.alive = False

    def commitSubTaskTransaction(self, reg: Region, time):
        if time not in self.timeRange:
            print(f"message: submit Task{self.id}-reg{reg.id} overtime! time:{time}")
            # 回滚任务状态
            self.subtask_status[reg.id] += 1
            if any(self.subtask_status.values()):
                self.Finished = False
        if time > self.timeRange.e:
            self.alive = False

    def rollbackSubTaskTransaction(self, reg: Region, time):
        """
        当robot在执行感知任务过程中无法完成感知任务，则需要回滚子任务事物
        """
        self.subtask_status[reg.id] += 1
        if any(self.subtask_status.values()):
            self.Finished = False
        if time > self.timeRange.e:
            self.alive = False

    def adequateSensor(self, sensor: Sensor):
        return sensor.category == self.__required_sensor.category  \
               and sensor.accuracy >= self.__required_sensor.accuracy
