import numbers
from abc import ABC, abstractmethod
from typing import List, Optional

from senseArea import Region, EuclideanDistance, Point
from sensor import Sensor
from RobotState import IdleState, MovingState, SensingState, BrokenState


def dataDiff(data1, data2):
    if bool(data1) ^ bool(data2):
        return 0.5
    if type(data1) != type(data2):
        raise TypeError("{}{} are different data type".format(type(data1), type(data2)))
    if isinstance(data1, numbers.Real):
        return abs(data1 - data2)
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

    def dissimilarity(self, other: 'RobotCategory') -> float:
        dis = dataDiff(self.move_mode, other.move_mode)
        dis += dataDiff(self.v, other.v)
        for key, val in self.physical_property.items():
            dis += dataDiff(val, self.physical_property.get(key))
        return dis

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
        :param tr:
        :param reg1:
        :param reg2:
        :param percentage:
        :return:
        """


class Robot:

    def __init__(self, rid, r_category, init_reg, init_loc):
        # static info
        self.id = rid
        self.C: RobotCategory = r_category
        self.init_reg = init_reg

        # state
        self.idleState = IdleState(self)
        self.movingState = MovingState(self)
        self.sensingState = SensingState(self)
        self.brokenState = BrokenState(self)
        self.state = self.idleState

        # dynamic location info
        self.location: Point = init_loc
        self.current_region: Region = init_reg

        # dynamic task info
        self.current_task_region: Optional[Region] = None
        self.current_cursor = 0

        # planed task info
        self.task_in_reg: List[List] = []
        self.sensor_in_reg: List[List[Region]] = []
        self.planned_path: List[Region] = [init_reg]
        # 约定；当submit后，更新为real_finish_time
        self.finish_time: List[float] = [0]
        self.ideal_time_used: List[float] = [0]
        self.ideal_moving_time: List[float] = [0]
        self.ideal_sensing_time: List[float] = [0]

    def __repr__(self):
        return "Robot({}, {}, {})".format(self.id, self.C, self.state)

    """ robot actions """
    def assignTask(self, reg, task, used_sensor):
        # state
        self.state.assignTask(reg, task, used_sensor)

    def cancelPlan(self, time, tr):
        # state
        self.state.cancelPlan(time, tr)

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
        self.ideal_sensing_time = self.sensingState[:cursor]

    def canFinishTaskInTime(self, time):
        if self.isFinishMissions:  # 如果已经完成所有任务，则返回True
            return True
        assert time == self.finish_time[self.current_cursor-1]
        if time + self.ideal_time_used[self.current_cursor] > self.finish_time[self.current_cursor]:
            return False
        else:
            return True

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

    def arrivalTime(self, reg: Region):
        move_time = self.C.interD(self.planned_path[-1], reg) / self.C.v
        return self.finish_time[-1] + move_time

    def possiblePlan(self, reg, task):
        """
        机器人在reg区域执行task的可能_时间点_和使用的传感器
        """
        adequate_sensors = set(filter(lambda s: task.adequateSensor(s), self.C.sensors))
        return ((self.idealFinishTime(reg, s), s) for s in adequate_sensors)

    def idealFinishTime(self, reg, sensor: Sensor):
        """
        机器人到reg区域使用sensor完成任务的理想时间，具体情况与robot状态有关
        :param reg: 目标区域
        :param sensor: 所使用的传感器
        :return: 理想完成 _时间点_
        """

        # 因为机器人执行任务的流程一定是从上一区域移动到此区域，之后再完成任务
        move_time = self.C.interD(self.planned_path[-1], reg) / self.C.v

        # 当机器人为sensingState时，不能并发分配任务
        if self.state == self.sensingState:
            return self.finish_time[-1] + self.C.intraD(reg) / self.C.v + move_time

        check_init = self.planned_path[-1] == self.init_reg
        if not check_init and move_time == 0 and sensor not in self.sensor_in_reg[reg]:
            # 如果机器人目的区域仍是机器人之前的区域，且之前没有使用该传感器，且不是初始状态
            # 则可以并行执行，理想完成时间点为之前的完成时间
            return self.finish_time[-1]
        return self.finish_time[-1] + self.C.intraD(reg) / self.C.v + move_time

    def moveDistance(self):
        dis = 0
        pre_reg = self.init_reg
        for reg in self.planned_path[2:]:
            dis += self.C.intraD(reg) + self.C.interD(pre_reg, reg)
        return dis
