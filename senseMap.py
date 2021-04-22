import math
import numbers
import itertools
import collections
from typing import *

import numpy as np

from senseArea import Region
from task import TimeSlot
from robot import RobotCategory

MapPoint = collections.namedtuple("MapPoint", "reg ts rc")
History = collections.namedtuple("History", "r_perf m_point")


class SenseMap:
    """
    senseMap[i, j, k]
    i in Reg, j in TS, k in RC
    """

    def __init__(self,
                 size,
                 regs,
                 ts,
                 rc,
                 pho=0.1,
                 sigma_noise=0.03,
                 kappa=0.3):
        self.size = size
        self.Regions: List[Region] = regs
        self.TS: List[TimeSlot] = ts
        self.RC: List[RobotCategory] = rc
        self.cellNum = self.size[0]*self.size[1]*self.size[2]

        self.__map: Dict[MapPoint, tuple] = {}
        self.__prior_map: Dict[MapPoint, int] = {}
        self.dimension = 3

        # base_algorithm parameters
        self.PHO = pho
        self.SIGMA_NOISE = sigma_noise
        self.UPDATE_KAPPA = kappa

        # updating attribute
        self.__history_len = int(self.cellNum * 0.8)
        self.__history: List[Optional[History]] = []
        self.update_times = 0

    @property
    def update_ratio(self):
        return self.update_times / self.cellNum

    def __repr__(self):
        return f"SenseMap({self.size})"

    def __stdKey(self, keys):
        if not isinstance(keys, tuple):
            raise TypeError(f"{type(self).__name__} indices must be tuple")

        for index, i_class in enumerate([Region, TimeSlot, RobotCategory]):
            if not isinstance(keys[index], int) or not isinstance(keys[index], i_class):
                raise IndexError(f"key[{index}] need int or {i_class.__name__}")

        keys = (x.getattr("id") for x in keys if type(x) != numbers.Integral)
        if any(i >= j for i, j in zip(keys, self.size)):
            raise IndexError("SenseMap index out of range")
        return MapPoint(*keys)

    def __getObj(self, key: MapPoint):
        return self.Regions[key.reg], self.TS[key.ts], self.RC[key.rc]

    def __getitem__(self, item):
        item = self.__stdKey(item)
        return self.__map.setdefault(item, (0, 0))

    def __setitem__(self, key, value):
        key = self.__stdKey(key)
        self.__map[key] = value

    def beginUpdating(self):
        old_values = self.__prior_map.values()
        p_range = max(old_values) - min(old_values)
        for key in itertools.product(range(x) for x in self.size):
            key = self.__stdKey(key)
            mu = self.__prior_map[key] / p_range * self.RC[key.rc].intraD(self.Regions[key.reg])
            sigma = self.matern(key, key)
            self[key] = (mu, sigma)

    def __new_update_cycle(self):
        for _, key in self.__history:
            self.__prior_map[key] = self.acquireFunction(key, self.UPDATE_KAPPA)
        self.__history.clear()
        self.update_times = 0

    def update(self, reg: Region, rt: float, rc: RobotCategory):
        t_ideal = rc.intraD(reg) / rc.v
        r_pref = t_ideal - rt / t_ideal
        try:
            ts = list(itertools.takewhile(lambda time_slot: rt in time_slot, self.TS))[0]
        except IndexError:
            raise ValueError(f"error real time {rt}")

        if len(self.__history) > self.__history_len:
            self.__new_update_cycle()
        self.__history.append(History(r_pref, MapPoint(reg, ts, rc)))

        self.__update_gaussian_process()
        self.update_times += 1

    def acquireFunction(self, key: tuple, kappa):
        return self[key][0] + kappa*self[key][1]

    def __update_gaussian_process(self):
        p_diff = np.array([r_perf - self.__prior_map[key]] for r_perf, key in self.__history)
        covariance = [[self.matern(x.m_point, y.m_point) for x in self.__history] for y in self.__history]
        cov_k_inv = np.linalg.inv(np.array(covariance) + self.SIGMA_NOISE * np.eye(len(self.__history)))

        # updating
        for key in self.__map.keys():
            k = np.array([self.matern(key, his.m_point) for his in self.__history])

            mu = self.__prior_map[key] + np.dot(k.T, np.dot(cov_k_inv, p_diff))
            sigma = self.matern(key, key) - np.dot(np.dot(k.T, cov_k_inv), k)
            self[key] = (mu, sigma)

    def dist(self, p1: MapPoint, p2: MapPoint):
        reg1, ts1, rc1 = self.__getObj(p1)
        reg2, ts2, rc2 = self.__getObj(p2)
        return reg1.dist(reg1) / reg1.len + ts1.dist(ts2) / ts1.len + rc1.dissimilarity(rc2)

    def matern(self, p1: MapPoint, p2: MapPoint):
        d = self.dist(p1, p2)
        return (1 + 5 ** 0.5 * d / self.PHO + 5 * d * d / (3 * self.PHO * self.PHO)) * math.exp(
            -5 ** 0.5 * d / self.PHO)


class MapCreator:
    pass
