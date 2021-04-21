import numbers
from abc import ABC, abstractmethod
from typing import List

from senseArea import Region, EuclideanDistance, Point
from sensor import Sensor
from RobotState import IdleState, PlaningState, MovingState, SensingState
from message import Message


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
    def intraD(self, reg: Region) -> float:  # todo sensor type affect
        """
        此类机器人在reg内执行感知任务的移动距离
        :param reg: 目标区域
        :return: 区域内移动距离
        """


class ConcreteRobot(RobotCategory):
    """
    具体的机器人类型
    """

    def intraD(self, reg: Region) -> float:
        pass

    def interD(self, reg1: Region, reg2: Region) -> float:
        pass


class Robot:

    def __init__(self, rid, r_category, init_reg, init_loc):
        self.id = rid
        self.C: RobotCategory = r_category
        self.init_reg = init_reg
        self.location: Point = init_loc

        # state
        self.idleState = IdleState(self)
        self.planingState = PlaningState(self)
        self.movingState = MovingState(self)
        self.sensingState = SensingState(self)
        self.brokenState = None
        self.state = self.idleState

        self.current_task_region = None
        self.current_cursor = 0

        self.task_in_reg = {}
        self.sensor_in_reg = {}

        self.planned_path: List[Region] = [init_reg]
        self.ideal_finish_time: List[float] = [0]

    def __repr__(self):
        return "Robot({}, {})".format(self.id, self.C)

    """ robot actions """
    def assignTask(self, reg, task, used_sensor):
        self.task_in_reg.setdefault(reg, []).append(task)
        self.sensor_in_reg.setdefault(reg, []).append(used_sensor)
        self.ideal_finish_time.append(self.arrivalTime(reg) + self.C.intraD(reg))

        # state
        self.state.assignTask()

    def cancelPlan(self):
        cancel_region = self.planned_path[self.current_cursor:]
        self.planned_path = self.planned_path[:self.current_cursor]
        self.ideal_finish_time = self.ideal_finish_time[:self.current_cursor]
        self.current_cursor -= 1
        for reg in cancel_region:
            del self.task_in_reg[reg]
            del self.sensor_in_reg[reg]
        # todo 循环reg
        # state
        self.state.cancelPlan()

    def executeMissions(self):
        self.current_cursor += 1
        self.current_task_region = self.planned_path[self.current_cursor]
        # state
        self.state.executeMissions()

    def submitTasks(self):
        self.current_cursor += 1
        if self.current_cursor < len(self.planned_path):
            self.current_task_region = self.planned_path[self.current_cursor]
        # state
        self.state.submitTask()

    def sense(self):
        # state
        self.state.sense()

    def broken(self):
        pass

    """ utility functions """
    def unfinishedTasks(self):
        unfinished = set()
        for reg in self.planned_path[self.current_cursor:]:
            for task in self.task_in_reg[reg]:
                unfinished.add(task)
        return unfinished

    def distBetweenRobot(self, r: 'Robot'):
        return EuclideanDistance(self.location, r.location)

    def arrivalTime(self, reg: Region):
        move_time = self.C.interD(self.planned_path[-1], reg) / self.C.v
        return self.ideal_finish_time[-1] + move_time

    def moveDistance(self):
        dis = 0
        pre_reg = self.init_reg
        for reg in self.planned_path[2:]:
            dis += self.C.intraD(reg) + self.C.interD(pre_reg, reg)
        return dis

