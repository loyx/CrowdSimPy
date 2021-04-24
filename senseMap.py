import math
import itertools
import collections
from typing import *

import numpy as np

from senseArea import Region
from task import TimeSlot
from robot import RobotCategory, Robot

MapPoint = collections.namedtuple("MapPoint", "reg ts rc")
History = collections.namedtuple("History", "r_perf m_point")


class SenseMap:
    """
    senseMap[i, j, k]
    i in Reg, j in TimeSlots, k in RobotCategories
    """

    def __init__(self,
                 map_size,
                 regions,
                 time_slots,
                 time_long,
                 robot_categories,
                 pho=0.1,
                 sigma_noise=0.03,
                 kappa=0.3):
        self.time_long = time_long
        self.size = map_size
        self.Regions: List[Region] = regions
        self.TimeSlots: List[TimeSlot] = time_slots
        self.RobotCategories: List[RobotCategory] = robot_categories
        self.cellNum = self.size[0] * self.size[1] * self.size[2]

        self.__map: Dict[MapPoint, tuple] = {}
        self.__prior_map: Dict[MapPoint, int] = {
            MapPoint(i, j, k): 0
            for i in range(self.size[0])
            for j in range(self.size[1])
            for k in range(self.size[2])
        }
        self.dimension = 3

        # base_algorithm parameters
        self.PHO = pho
        self.SIGMA_NOISE = sigma_noise
        self.UPDATE_KAPPA = kappa

        # updating attribute
        self.__history_len = int(self.cellNum * 0.8)
        self.__history: List[Optional[History]] = []
        self.update_times = 0

    def __repr__(self):
        return f"SenseMap(Size:{self.size}, Update:{self.update_times})"

    """ access SenseMap """

    def __stdKey(self, keys):
        if not isinstance(keys, tuple):
            raise TypeError(f"{type(self).__name__} indices must be tuple")

        for index, i_class in enumerate([Region, TimeSlot, RobotCategory]):
            if not isinstance(keys[index], int) and not isinstance(keys[index], i_class):
                raise IndexError(f"key[{index}] need int or {i_class.__name__}")

        keys = tuple(x.getattr("id") if type(x) != int else x for x in keys)
        if any(i >= j for i, j in zip(keys, self.size)):
            raise IndexError("SenseMap index out of range")
        return MapPoint(*keys)

    def __getitem__(self, item):
        item = self.__stdKey(item)
        return self.__map.setdefault(item, (0, 0))

    def __setitem__(self, key, value):
        key = self.__stdKey(key)
        self.__map[key] = value

    """ sensMap info """

    @property
    def update_ratio(self):
        return self.update_times / self.cellNum

    """ senseMap action """

    def beginUpdating(self):
        old_values = self.__prior_map.values()
        p_range = max(old_values) - min(old_values)
        if p_range == 0:
            p_range = 1
        for key in itertools.product(*(range(x) for x in self.size)):
            key = self.__stdKey(key)
            robot_category = self.RobotCategories[key.rc]  # todo 优化：简化操作，放在robot类里
            mu = self.__prior_map[key] / p_range * robot_category.intraD(self.Regions[key.reg]) / robot_category.v
            sigma = self.__matern(key, key)
            self[key] = (mu, sigma)

    def update(self, reg: Region, rt: float, r: Robot):
        t_ideal = r.C.intraD(reg) / r.C.v

        # senseMap 的Update发生在robot submit之后，此时cursor指向下一个目标任务
        # 因此上一任务的实际用时为 [cursor-1] - [cursor-2]
        assert rt == r.finish_time[r.current_cursor - 1]
        real_used_time = rt - r.finish_time[r.current_cursor - 2]
        r_pref = t_ideal - real_used_time / t_ideal

        # 因为感知信息图记录的是一个时间周期内的情况（例如，1天24h内）
        # 而robot的real time是从0起的rt秒，因此需要对时间周期长度取余
        std_real_time = rt % self.time_long
        try:
            ts = list(itertools.takewhile(lambda time_slot: std_real_time in time_slot, self.TimeSlots))[0]
        except IndexError:
            raise ValueError(f"error real time {rt}")

        self.__update_gaussian_process()
        self.update_times += 1

        if len(self.__history) > self.__history_len:
            self.__new_update_cycle()
        self.__history.append(History(r_pref, MapPoint(reg, ts, r.C)))

    def acquireFunction(self, key: tuple, kappa):
        return self[key][0] + kappa * self[key][1]

    """ utility functions """

    def __new_update_cycle(self):
        for _, key in self.__history:
            self.__prior_map[key] = self.acquireFunction(key, self.UPDATE_KAPPA)
        self.__history.clear()
        self.update_times = 0

    def __update_gaussian_process(self):
        p_diff = np.array([r_perf - self.__prior_map[key]] for r_perf, key in self.__history)
        covariance = [[self.__matern(x.m_point, y.m_point) for x in self.__history] for y in self.__history]
        cov_k_inv = np.linalg.inv(np.array(covariance) + self.SIGMA_NOISE * np.eye(len(self.__history)))

        # updating
        for key in self.__map.keys():
            k = np.array([self.__matern(key, his.m_point) for his in self.__history])

            mu = self.__prior_map[key] + np.dot(k.T, np.dot(cov_k_inv, p_diff))
            sigma = self.__matern(key, key) - np.dot(np.dot(k.T, cov_k_inv), k)
            self[key] = (mu, sigma)

    def __getObj(self, key: MapPoint):
        return self.Regions[key.reg], self.TimeSlots[key.ts], self.RobotCategories[key.rc]

    def __dist(self, p1: MapPoint, p2: MapPoint):
        # todo 优化：距离各部分的占比和归一化
        reg1, ts1, rc1 = self.__getObj(p1)
        reg2, ts2, rc2 = self.__getObj(p2)
        return reg1.dist(reg1) / reg1.area + ts1.dist(ts2) / ts1.len + rc1.dissimilarity(rc2)

    def __matern(self, p1: MapPoint, p2: MapPoint):
        d = self.__dist(p1, p2)
        return (1 + 5 ** 0.5 * d / self.PHO + 5 * d * d / (3 * self.PHO * self.PHO)) * math.exp(
            -5 ** 0.5 * d / self.PHO)


class MapCreator:
    pass
