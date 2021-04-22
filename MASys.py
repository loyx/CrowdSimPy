import itertools
import operator
import queue
from abc import ABC, abstractmethod
from functools import reduce
from operator import attrgetter, methodcaller
from typing import List, Optional, Tuple

from senseArea import SenseArea, Region
from senseMap import SenseMap
from task import Task, TimeRange, TimeSlot
from robot import Robot, RobotCategory
from message import Message, FeedBack


class MACrowdSystem:

    def __init__(self,
                 sense_area: SenseArea,
                 grid_granularity,
                 sense_time: TimeRange,
                 time_granularity,
                 robot_categorise,
                 base_algorithm,
                 repair_k=1
                 ):
        self.__robots: List[Optional[Robot]] = []
        self.__tasks: List[Optional[Task]] = []
        self.__base_algorithm: BaseAlgorithm = base_algorithm
        self.__repair_k = repair_k

        self.__finished_tasks = []

        self.sense_area = sense_area
        self.grid_granularity = grid_granularity
        self.Regions: List[Region] = self.sense_area.grid(self.grid_granularity)

        self.sense_time = sense_time
        self.time_granularity = time_granularity
        self.TS: List[TimeSlot] = sense_time.discretize(self.time_granularity)

        self.RC: List[RobotCategory] = robot_categorise

        map_size = (len(self.Regions), len(self.TS), len(self.RC))
        self.senseMap = SenseMap(map_size, self.Regions, self.TS, self.RC)

    def run(self):
        # self.senseMap.creation()
        self.senseMap.beginUpdating()

        # self-repairing task allocation base_algorithm
        self.__base_algorithm.new_allocationPlan(self.__tasks, self.__robots, self.senseMap)
        self.__base_algorithm.allocationTasks()
        while len(self.__finished_tasks) != len(self.__tasks):
            # 执行感知任务
            message = yield from self.execMissions()
            if self.needRepairing(message):
                # 构建新的T和R
                k = int(self.__repair_k * len(self.__robots))
                new_tasks, new_robots = self.constructNewPlan(message, k)
                yield FeedBack(1, new_robots)
                self.__base_algorithm.new_allocationPlan(new_tasks, new_robots, self.senseMap)
                self.__base_algorithm.allocationTasks()

    def execMissions(self):
        for r in self.__robots:
            r.executeMissions()
        message = yield
        while True:
            if message.status_code == 0:
                self.senseMap.update(message.region, message.real_time, message.robot_category)
                if self.senseMap.update_ratio >= 0.8:
                    return message
            else:
                return message
            message = yield FeedBack(0)

    def needRepairing(self, message: Message):
        if message.status_code != 0 and self.senseMap.update_ratio > 0.8:
            return True
        else:
            return False

    def constructNewPlan(self, message, k) -> Tuple[List[Task], List[Robot]]:
        assert message.status_code != 0
        if k == len(self.__robots):
            new_task = list(filter(lambda t: not t.isFinished, self.__tasks))
            new_robot = list(filter(lambda robot: not robot.isBroken, self.__robots))
            return new_task, new_robot

        target_r = None
        for r in self.__robots:
            if r.id == message.robot_id:
                target_r = r
                break
        most_near = queue.PriorityQueue(k)
        for r in self.__robots:
            most_near.put((target_r.distBetweenRobot(r), r))

        new_robots: List[Robot] = [t[1] for t in most_near.queue]
        for r in new_robots:
            r.cancelPlan()
        new_tasks = list(reduce(operator.or_, [x.unfinishedTasks() for x in new_robots]))
        return new_tasks, new_robots

    def decomposeTask(self, task: Task):
        assert not task.TR
        for reg in self.Regions:
            if reg.center in task.area:
                task.TR.append(reg)
        task.TR.sort(key=attrgetter('id'))
        task.finished_reg = {reg.id: False for reg in task.TR}

    def publishTask(self, task):
        self.decomposeTask(task)
        self.__tasks.append(task)

    def registerRobot(self, robot):
        self.__robots.append(robot)


class BaseAlgorithm(ABC):

    def __init__(self, gamma=1):
        self._robots: List[Robot] = []
        self._tasks: List[Task] = []
        self._sense_map: Optional[SenseMap] = None
        self._kappa = None

        self.allocationPlan = {}
        self.GAMMA = gamma

    def new_allocationPlan(self, tasks, robots, s_map, kappa=0.03):
        self._tasks = tasks
        self._robots = robots
        self._sense_map = s_map
        self._kappa = kappa
        self.allocationPlan.clear()
        assert not self.allocationPlan

    @abstractmethod
    def allocationTasks(self):
        """
        任务分配基算法，就地分配任务给机器人
        """

    def totalCov(self):
        cov = 0
        for task in self._tasks:
            s = sum(self.allocationPlan.setdefault((task.id, reg.id, r.id), 0)
                    for reg in task.TR
                    for r in self._robots)
            cov += s / len(task.TR) / self.GAMMA
        return cov

    def robotDis(self):
        return sum(map(methodcaller('moveDistance'), self._robots))


class GreedyBaseAlgor(BaseAlgorithm):

    def __init__(self, gamma=1, thetas=(1, 1, 1), lambdas=(1, 1, 1)):
        super().__init__(gamma)
        self.THETAS = thetas
        self.LAMBDAS = lambdas

    def allocationTasks(self):
        task_in_reg = {}
        for task in self._tasks:
            if task.isFinished:
                continue
            for reg in task.TR:
                if not task.finished_reg[task.id]:
                    task_in_reg.setdefault(reg, []).append(task)

        for reg, tasks in task_in_reg.items():
            task: Task
            for task in tasks:
                u_max = 0
                r_max = None
                s_select = None
                for rob in self._robots:
                    finish_time, select_sensors = min(rob.possiblePlan(reg, task))
                    sample_times = self.allocationPlan.setdefault((task, reg, rob), 0)
                    if finish_time not in task.timeRange or not select_sensors and sample_times < self.GAMMA:
                        continue
                    u = self.DeltaUtility(reg, rob, finish_time)
                    if u > u_max:
                        u_max = u
                        r_max = rob
                        s_select = select_sensors.pop()
                if not r_max:
                    r_max.assignTask(reg, task, s_select)
                    self.allocationPlan[(task, reg, r_max)] += 1

    def DeltaUtility(self, reg: Region, r: Robot, at: int):
        f1 = self.THETAS[0] * 1 / self.LAMBDAS[0]
        f2 = self.THETAS[1] * (r.C.interD(r.planned_path[-1], reg) + r.C.intraD(reg)) / self.LAMBDAS[1]

        try:
            ts = list(itertools.takewhile(lambda t: at in t, self._sense_map.TS))[0]
        except IndexError:
            raise ValueError(f"error arrival time {at}")
        f3 = self.THETAS[2] * self._sense_map.acquireFunction((reg, ts, r.C), self._kappa) / self.LAMBDAS[2]
        return f1 + f2 + f3
