import queue
import collections
from typing import Dict

from robot import Robot
from realWorld import RealWorld

Event = collections.namedtuple("Event", "time robot action")


def physicalRobot(robot: Robot, start_time=0):
    assert robot.state == robot.idleState
    # robot 预激后，由MASys负责分配任务，并启动所有robot
    # 因而下一个yield时，robot的状态应该为MovingState
    time = yield Event(start_time, robot, "start moving")
    # 所有的robot都应从0开始计时，由Simulator估计moving时间

    assert robot.state == robot.movingState
    while not robot.isFinishMissions:  # todo brokenState
        # 因为robot的激活和规划由MASys完成，因而此循环里只有moving和sensing状态
        # 同时，我们应当假设idle，planing状态不花费时间

        time = yield Event(time, robot, "start sensing")
        assert robot.state == robot.sensingState
        # robot.sense()  # robot到到达目标地点，开始感知数据
        # 返回SensingState Robot，有Simulator估计Sensing时间

        time = yield Event(time, robot, "start moving")
        assert robot.state == robot.movingState
        # robot.submitTasks()  # robot完成感知提交任务
        # 返回MovingState Robot，由Simulator估计下一个Moving的时间

        # robot 以 (移动-感知) 为一个操作周期，直至分配给其的任务执行完成


class Simulator:

    def __init__(self, p_robots):
        self.events = queue.PriorityQueue()
        self.p_robots: Dict = p_robots
        self.realWorld = RealWorld()

    def run(self, end_time):
        # init

        sim_time = 0
        robot: Robot
        while sim_time < end_time:
            if self.events.empty():
                print("*** end of events ***")
                break

            sim_time, robot, action = self.events.get()

            # 这些操作发生在状态转化的那个瞬间
            if robot.state == robot.movingState:
                robot.sense()
            elif robot.state == robot.sensingState:
                robot.submitTasks()

            print("time:", sim_time, robot.id * '  ', robot)
            next_time = sim_time + self.realWorld.compute_duration(robot)
            active_p_robot = self.p_robots[robot.id]
            try:
                next_event = active_p_robot.send(next_time)
            except StopIteration:
                del self.p_robots[robot.id]
            else:
                self.events.put(next_event)
        else:
            print(f"*** end of simulation time: {self.events.qsize()} events pending ***")


def main():
    pass


if __name__ == '__main__':
    main()
