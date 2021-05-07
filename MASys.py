import operator
import queue
from abc import ABC, abstractmethod
from functools import reduce
from operator import attrgetter, methodcaller
from typing import List, Optional, Tuple

from senseArea import SenseArea, Region
from senseMap import SenseMap
from task import Task, TimeSlot, TimeCycle
from robot import Robot, RobotCategory
from message import Message, FeedBack
from resultDisplay import pltMASys


class MACrowdSystem:

    def __init__(self,
                 sense_area: SenseArea,
                 grid_granularity,
                 sense_time: TimeCycle,
                 time_granularity,
                 robot_categorise,
                 base_algorithm,
                 repair_k=1,
                 info_save=False
                 ):
        self.robots: List[Optional[Robot]] = []
        self.tasks: List[Optional[Task]] = []
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

        self.info_save = info_save

    @property
    def TaskNums(self):
        return sum(len(t.TR) for t in self.tasks)

    """ actions """
    def publishTask(self, task):
        self.__decomposeTask(task)
        self.tasks.append(task)

    def registerRobot(self, robot):
        self.robots.append(robot)

    def run(self):
        # self.senseMap.creation()
        self.senseMap.beginUpdating()
        print("### MASys: senseMap ready ###")

        # self-repairing task allocation base_algorithm
        self.__base_algorithm.new_allocationPlan(self.tasks, self.robots, self.senseMap)
        self.__base_algorithm.allocationTasks()

        # print分配结果
        cov = self.__base_algorithm.totalCov()
        r_dis = self.__base_algorithm.robotDis()
        print("### MASys: finished allocation tasks ###")
        print(f"### MASys: ideal cov: {cov}, ideal robot dis: {r_dis} ###")
        # plt
        plt = pltMASys(self, False, self.info_save)
        try:
            next(plt)
        except StopIteration:
            pass

        while len(self.__finished_tasks) != len(self.tasks):
            # 执行感知任务
            print("\n### MASys: start execution ###")
            message = yield from self.__execMissions()
            print(f"### something wrong: {message} ###")
            if self.__needRepairing(message):
                # 自修复前先画图
                plt = pltMASys(self, True, self.info_save)
                next(plt)

                # 构建新的T和R
                print("### MASys: start self repairing ###")
                k = int(self.__repair_k * len(self.robots))
                new_tasks, new_robots = self.__constructNewPlan(message, k)
                yield FeedBack(1, new_robots)
                self.__base_algorithm.new_allocationPlan(new_tasks, new_robots, self.senseMap)
                self.__base_algorithm.allocationTasks()

                # print 修复结构
                cov = self.__base_algorithm.totalCov()
                r_dis = self.__base_algorithm.robotDis()
                print("### MASys: finished allocation tasks ###")
                print(f"### MASys: ideal cov: {cov}, ideal robot dis: {r_dis} ###")
                # 画图
                try:
                    next(plt)
                except StopIteration:
                    pass

    """ utility functions """
    def __execMissions(self):
        for r in self.robots:
            r.executeMissions()
        message = yield
        while True:
            if message.status_code == 0:
                self.senseMap.update(message.region, message.real_time, message.robot)
                if self.senseMap.update_ratio >= 0.8:
                    return message
            else:
                return message
            message = yield FeedBack(0)

    def __needRepairing(self, message: Message):
        if message.status_code != 0 or self.senseMap.update_ratio > 0.8:
            return True
        else:
            return False

    def __constructNewPlan(self, message, k) -> Tuple[List[Task], List[Robot]]:
        assert message.status_code != 0
        if k == len(self.robots):
            new_tasks = list(filter(lambda t: not t.canFinish, self.tasks))
            new_robots = list(filter(lambda robot: not robot.isBroken, self.robots))
        else:
            target_r = None
            for r in self.robots:
                if r.id == message.robot_id:
                    target_r = r
                    break
            assert target_r == message.robot

            most_near = queue.PriorityQueue(k)
            for r in self.robots:
                most_near.put((target_r.distBetweenRobot(r), r))

            new_robots: List[Robot] = [t[1] for t in most_near.queue]
            new_tasks = list(reduce(operator.or_, [x.unfinishedTasks() for x in new_robots]))

        # 取消自修复涉及到的机器人的任务计划
        for r in new_robots:
            r.cancelPlan(message.real_time, self.Regions)
        return new_tasks, new_robots

    def __decomposeTask(self, task: Task):
        assert not task.TR
        for reg in self.Regions:
            if reg.center in task.area:
                task.TR.append(reg)
        task.TR.sort(key=attrgetter('id'))
        task.subtask_status = {reg.id: self.__base_algorithm.GAMMA for reg in task.TR}


class BaseAlgorithm(ABC):

    def __init__(self, gamma=1):
        self.robots: List[Robot] = []
        self.tasks: List[Task] = []
        self.sense_map: Optional[SenseMap] = None
        self.kappa = None

        self.allocationPlan = {}
        self.sampleRecord = {}
        self.GAMMA = gamma

    def new_allocationPlan(self, tasks, robots, s_map, kappa=0.03):
        self.tasks = tasks
        self.robots = robots
        self.sense_map = s_map
        self.kappa = kappa
        self.allocationPlan.clear()
        self.sampleRecord.clear()
        assert not self.allocationPlan

    @abstractmethod
    def allocationTasks(self):
        """
        任务分配基算法，就地分配任务给机器人
        """

    def totalCov(self):
        cov = 0
        for task in self.tasks:
            s = sum(self.allocationPlan.get((task.id, reg.id, r.id), 0)
                    for reg in task.TR
                    for r in self.robots)
            cov += s / len(task.TR) / self.GAMMA
        cov_rate = cov / len(self.tasks)
        assert cov_rate <= 1.0
        return cov_rate

    def robotDis(self):
        return sum(map(methodcaller('moveDistance'), self.robots))


class GreedyBaseAlgorithm(BaseAlgorithm, ABC):

    def __init__(self, gamma=1, thetas=(1, 1, 1), lambdas=(1, 1, 1)):
        super().__init__(gamma)
        self.THETAS = thetas
        self.LAMBDAS = lambdas

    @abstractmethod
    def allocationTasks(self):
        pass

    def DeltaUtility(self, reg: Region, r: Robot, at: int):
        f1 = self.THETAS[0] * 1 / self.LAMBDAS[0]
        f2 = self.THETAS[1] * (r.planned_distance[-1] + r.taskDistance(reg)) / self.LAMBDAS[1]

        try:
            ts = list(filter(lambda t: at in t, self.sense_map.TimeSlots))[0]
        except IndexError:
            raise ValueError(f"error arrival time {at}")
        f3 = self.THETAS[2] * self.sense_map.acquireFunction((reg, ts, r.C), self.kappa) / self.LAMBDAS[2]
        return f1 - f2 + f3


class RobotOrientAlgorithm(GreedyBaseAlgorithm):

    def allocationTasks(self):
        record_u = {}
        for task in self.tasks:
            if task.canFinish:  # 可以完成的任务无需分配
                continue
            for reg in task.TR:
                if task.subtask_status[reg.id] == 0:  # 可以完成的子任务无需分配
                    continue
                for r in self.robots:
                    finish_time, select_sensor = min(r.possiblePlan(reg, task))
                    if finish_time not in task.timeRange or not select_sensor:
                        continue
                    record_u[task, reg, r] = self.DeltaUtility(reg, r, finish_time), select_sensor
        test = [(value[0], key) for key, value in record_u.items()]
        test.sort(key=lambda x: x[0], reverse=True)

        while True:

            # 贪心选取一种分配方案
            max_u_s = None
            max_key = None
            for key, u_s in record_u.items():
                if self.sampleRecord.get((key[0].id, key[1].id), 0) >= self.GAMMA:
                    continue
                if not max_u_s or u_s > max_u_s:
                    max_u_s = u_s
                    max_key = key
            if max_key is None:
                break
            task, reg, robot_star = max_key
            robot_star: Robot
            _, sensor = max_u_s

            # 分配任务
            robot_star.assignTask(reg, task, sensor)
            self.allocationPlan[task.id, reg.id, robot_star.id] = 1
            try:
                self.sampleRecord[task.id, reg.id] += 1
            except KeyError:
                self.sampleRecord[task.id, reg.id] = 1

            # 清除该记录
            del record_u[task, reg, robot_star]

            # 更新该机器人与其他任务的$\Delta U$
            for task in self.tasks:
                if task.canFinish:
                    continue
                for reg in task.TR:
                    if task.subtask_status[reg.id] == 0:
                        continue
                    if self.allocationPlan.get((task.id, reg.id, robot_star.id), 0):
                        continue
                    finish_time, select_sensor = min(robot_star.possiblePlan(reg, task))
                    if finish_time not in task.timeRange or not select_sensor:
                        continue
                    record_u[task, reg, robot_star] = self.DeltaUtility(reg, robot_star,
                                                                        finish_time), select_sensor


class TaskOrientAlgorithm(GreedyBaseAlgorithm):

    def allocationTasks(self):
        task_in_reg = {}
        for task in self.tasks:
            if task.canFinish:
                continue
            for reg in task.TR:
                if task.subtask_status[reg.id] == 0:
                    continue
                task_in_reg.setdefault(reg, []).append(task)

        for reg, tasks in task_in_reg.items():
            task: Task
            for task in tasks:
                u_max = None
                r_max = None
                s_select = None
                for rob in self.robots:
                    finish_time, select_sensors = min(rob.possiblePlan(reg, task))
                    sample_times = self.allocationPlan.get((task, reg, rob), 0)
                    if finish_time not in task.timeRange or not select_sensors or sample_times >= self.GAMMA:
                        continue
                    u = self.DeltaUtility(reg, rob, finish_time)
                    if u_max is None or u > u_max:
                        u_max = u
                        r_max = rob
                        s_select = select_sensors
                if r_max:
                    r_max.assignTask(reg, task, s_select)
                    ap = (task.id, reg.id, r_max.id)
                    self.allocationPlan[ap] = self.allocationPlan.get(ap, 0) + 1
