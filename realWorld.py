import random

from robot import Robot
from concreteRobot import UV, UAV, SmallUV


class RealWorld:
    SIM_ROB = [UAV, UV, SmallUV]

    def __init__(self, region_size, thresholds, thetas, moving_affect=0.2):
        rd = random.random
        self.reg_info = [(rd(), rd(), rd()) for _ in range(region_size)]
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
            return ideal_time*(1-reg_rate)*self.thetas[index]
        else:
            ideal_time = robot.ideal_moving_time[robot.current_cursor]
            return ideal_time*random.uniform(1, self.moving_affect)

    def canSense(self, robot: Robot) -> bool:
        for i, cls in enumerate(RealWorld.SIM_ROB):
            if isinstance(robot.C, cls):
                if self.reg_info[robot.current_task_region.id][i] < self.thresholds[i]:
                    return False
                else:
                    return True
        raise RuntimeError("unknown RobotCategory")
