import functools
import numbers
from abc import ABC, abstractmethod
from typing import List, Optional

from senseArea import Region, EuclideanDistance, Point
from sensor import Sensor
from RobotState import IdleState, MovingState, SensingState, BrokenState
from task import Task


def dataDiff(data1, data2):
    if bool(data1) ^ bool(data2):
        return 0.5
    if isinstance(data1, numbers.Real):
        return abs(data1 - data2)
    elif type(data1) != type(data2):
        raise TypeError("{}{} are different data type".format(type(data1), type(data2)))
    elif isinstance(data1, str):
        return int(data1 == data2)
    else:
        raise NotImplementedError(type(data1))


class RobotCategory(ABC):

    def __init__(self, rc_id, category, sensors, move_mode, v, physical_property):
        self.id = rc_id
        self.category: str = category
        self.sensors: List[Sensor] = sensors
        self.move_mode: str = move_mode
        self.v: float = v
        self.physical_property: dict = physical_property

    def __hash__(self):
        """
        python官方文档建议应先实现__eq__方法，这里使用来自Object的默认__eq__，即对比id()。
        实现__hash__的目标是使RobotCategory可散列，从而可以使用lru_cache，优化dissimilarity速度。
        本__hash__只返回hash(self.id)，是因为每一个id只对应一个RobotCategory实例，
        不采用文档建议方法是因为RobotCategory类 类似于 “([], [], ...)”
        """
        return hash(self.id)

    @functools.lru_cache
    def dissimilarity(self, other: 'RobotCategory') -> float:
        dis = dataDiff(self.move_mode, other.move_mode)
        dis += dataDiff(self.v, other.v)
        for key, val in self.physical_property.items():
            other_val = self.physical_property.get(key)
            dis += dataDiff(val, other_val)
        return dis

    def __repr__(self):
        cls = type(self)
        return f"{cls.__name__}(id:{self.id}, category:{self.category}, move mode:{self.move_mode})"

    @abstractmethod
    def interD(self, reg1: Region, reg2: Region) -> float:
        """
        此类机器人从reg1移动到reg2之间的距离
        :param reg1: 出发区域
        :param reg2: 目标区域
        :return: 跨区域移动距离
        """

    @abstractmethod
    def intraD(self, reg: Region) -> float:  # todo 优化：sensor type affect
        """
        此类机器人在reg内执行感知任务的移动距离
        :param reg: 目标区域
        :return: 区域内移动距离
        """

    @abstractmethod
    def getLocation(self, reg1, reg2, percentage, tr: List[Region]) -> Region:
        """
        该函数是robot在movingState时，根据移动距离的百分比返回所在区域
        """


class Robot:

    def __init__(self, rid, r_category, init_reg):
        # static info
        self.id = rid
        self.C: RobotCategory = r_category
        self.init_reg: Region = init_reg

        # state
        self.idleState = IdleState(self)
        self.movingState = MovingState(self)
        self.sensingState = SensingState(self)
        self.brokenState = BrokenState(self)
        self.state = self.idleState

        # dynamic location info
        self.location: Point = self.init_reg.randomLoc()
        self.current_region: Region = init_reg

        # dynamic task info
        self.current_task_region: Optional[Region] = None
        self.current_cursor = 0
        """
            current_cursor指示当前执行的任务
                           current_cursor
                                 V
            [finished_reg, ..., reg, ..., unfinished_reg]
        """

        # 因为current_cursor初始值为0，因此以下都需要一个占位
        # planed task info
        self.task_in_reg: List[List] = [[None]]
        self.sensor_in_reg: List[List] = [[None]]
        self.planned_path: List[Region] = [init_reg]
        self.planned_distance: List[float] = [0]
        # 约定；当submit后，更新为real_finish_time
        self.finish_time: List[float] = [0]
        self.ideal_time_used: List[float] = [0]
        self.ideal_moving_time: List[float] = [0]
        self.ideal_sensing_time: List[float] = [0]

    def __repr__(self):
        return "Robot(id:{:}, c:{}, {}, loc:Region{})" \
            .format(self.id, type(self.C).__name__, self.state, self.init_reg.id)

    def __str__(self):
        return f"Robot({self.id:>2}, {self.state})"

    """ robot actions """

    def assignTask(self, reg, task, used_sensor):
        # state
        self.state.assignTask(reg, task, used_sensor)

    def cancelPlan(self, time, regions):
        # state
        self.state.cancelPlan(time, regions)

    def executeMissions(self):
        # state
        self.state.executeMissions()

    def submitTasks(self, time):
        # state
        self.state.submitTask(time)

    def sense(self):
        # state
        self.state.sense()

    def broken(self):
        # state
        self.state.broken()

    """ utility functions """

    def clearRecord(self, cursor):
        self.planned_path = self.planned_path[:cursor]
        self.finish_time = self.finish_time[:cursor]
        self.task_in_reg = self.task_in_reg[:cursor]
        self.sensor_in_reg = self.sensor_in_reg[:cursor]  # use GC
        self.ideal_time_used = self.ideal_time_used[:cursor]
        self.ideal_moving_time = self.ideal_moving_time[:cursor]
        self.ideal_sensing_time = self.ideal_sensing_time[:cursor]
        self.planned_distance = self.planned_distance[:cursor]

    def canFinishTaskInTime(self, time):
        if self.isFinishMissions:  # 如果已经完成所有任务，则返回True
            return True
        assert time == self.finish_time[self.current_cursor - 1]
        if time + self.ideal_time_used[self.current_cursor] > self.finish_time[self.current_cursor]:
            return False
        else:
            return True

    @property
    def currentTasks(self):
        return self.task_in_reg[self.current_cursor]

    @property
    def isFinishMissions(self):
        return self.current_cursor >= len(self.planned_path) and self.state == self.idleState

    @property
    def isBroken(self):
        return self.state == self.brokenState

    def unfinishedTasks(self):
        unfinished = set()
        for tasks in self.task_in_reg[self.current_cursor:]:
            for t in tasks:
                unfinished.add(t)
        return unfinished

    def distBetweenRobot(self, r: 'Robot'):
        return EuclideanDistance(self.location, r.location)

    def possiblePlan(self, reg, task):
        """
        机器人在reg区域执行task的可能_时间点_和使用的传感器
        """
        adequate_sensors = set(filter(lambda s: task.adequateSensor(s), self.C.sensors))
        return ((self.idealFinishTime(reg, s, task), s) for s in adequate_sensors)

    def idealFinishTime(self, reg, sensor: Sensor, task: Task):
        """
        机器人到reg区域使用sensor完成任务的理想时间，具体情况与robot状态有关
        :param task: 目标任务
        :param reg: 目标区域
        :param sensor: 所使用的传感器
        :return: 理想完成 _时间点_
        """

        # 因为机器人执行任务的流程一定是从上一区域移动到此区域，之后再完成任务
        move_time = self.C.interD(self.planned_path[-1], reg) / self.C.v

        if not self.state == self.sensingState and not self.current_task_region \
                and (move_time == 0 and sensor not in self.sensor_in_reg[-1]):
            # 判断1：机器人为sensingState时，或位于初始状态，不能并发分配任务
            # 判断2；如果机器人目的区域仍是机器人之前的区域，且之前没有使用该传感器，则可以并行执行
            # 理想完成时间点为之前的完成时间
            return self.finish_time[-1]

        arrival_time = self.finish_time[-1] + move_time
        # 当机器人到达目标地点时，任务尚未开始，此时原地等待直至任务开始。
        return max(arrival_time, task.timeRange.s) + self.C.intraD(reg) / self.C.v

    def taskDistance(self, reg: Region):
        return self.C.interD(self.planned_path[-1], reg) + self.C.intraD(reg)

    def moveDistance(self):
        return self.planned_distance[-1]
