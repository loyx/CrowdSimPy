import random

from robot import Robot
from concreteRobot import UV, UAV, SmallUV


class RealWorld:
    SIM_ROB = [UAV, UV, SmallUV]

    def __init__(self, region_size, thresholds, thetas, moving_affect=0.2):
        rd = random.random
        # reg_info 代表每一reg 感知的容易程度，越大越容易感知
        self.reg_info = [(rd(), rd(), rd()) for _ in range(region_size)]
        # thresholds 表示robot不能感知reg的概率
        self.thresholds = thresholds
        self.thetas = thetas
        self.moving_affect = 1 + moving_affect

    def compute_duration(self, robot: Robot) -> float:
        index = None
        for i, cls in enumerate(RealWorld.SIM_ROB):
            if isinstance(robot.C, cls):
                index = i
                break

        if robot.state == robot.sensingState:
            ideal_time = robot.ideal_sensing_time[robot.current_cursor]
            reg_rate = self.reg_info[robot.current_task_region.id][index]
            sense_time = ideal_time * (12 ** (-3 * reg_rate + 0.3) + 0.9975) * self.thetas[index]
            return sense_time
        elif robot.state == robot.movingState:
            ideal_time = robot.ideal_moving_time[robot.current_cursor]
            rate = random.uniform(1, self.moving_affect)
            if rate < 0.6*self.moving_affect+0.4:
                rate = 1
            moving_time = ideal_time * rate
            return moving_time
        elif robot.state == robot.idleState:
            return 0
        else:
            raise RuntimeError(f"error state{robot.state} when compute duration")

    def canSense(self, robot: Robot) -> bool:
        for i, cls in enumerate(RealWorld.SIM_ROB):
            if isinstance(robot.C, cls):
                if self.reg_info[robot.current_task_region.id][i] < self.thresholds[i]:
                    return False
                else:
                    return True
        raise RuntimeError("unknown RobotCategory")
