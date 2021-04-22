import heapq
import collections
from typing import Dict, List

from robot import Robot
from MASys import MACrowdSystem
from realWorld import RealWorld
from message import Message, FeedBack

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

    def __init__(self, p_robots, ma_sys):
        self.events: List[Event] = []
        self.p_robots: Dict = p_robots
        self.realWorld = RealWorld()
        self.MASys: MACrowdSystem = ma_sys

    def run(self, end_time):
        # init

        # 预激robot
        for p_robot in self.p_robots.values():
            first_event = next(p_robot)
            # self.events.put(first_event)
            heapq.heappush(self.events, first_event)

        # 预激MASys，并分配任务, 启动robot
        sim_sys = self.MASys.run()
        next(sim_sys)

        # start simulation
        sim_time = 0
        robot: Robot
        while sim_time < end_time:
            # if self.events.empty():
            if len(self.events):
                print("*** end of events ***")
                break

            # sim_time, robot, action = self.events.get()
            sim_time, robot, action = heapq.heappop(self.events)

            # 这些操作发生在状态转化的那个瞬间  # todo brokenState
            if robot.state == robot.movingState:
                if self.realWorld.canSense(robot):
                    robot.sense()
                    feed_back = FeedBack(0)
                else:
                    message = Message(3, robot.id, robot.C, robot.current_region, sim_time)
                    feed_back: FeedBack = sim_sys.send(message)
            elif robot.state == robot.sensingState:
                robot.submitTasks()  # todo other case
                message = Message(0, robot.id, robot.C, robot.current_region, sim_time)
                feed_back: FeedBack = sim_sys.send(message)
            else:
                raise RuntimeError("error robot")

            if feed_back.status_code == 0:
                print("time:", sim_time, robot.id * '  ', robot)
                next_time = sim_time + self.realWorld.compute_duration(robot)
                active_p_robot = self.p_robots[robot.id]
                try:
                    next_event = active_p_robot.send(next_time)
                except StopIteration:
                    del self.p_robots[robot.id]
                else:
                    # self.events.put(next_event)
                    heapq.heappush(self.events, next_event)
            elif feed_back.status_code == 1:
                need_repair_robots: List[Robot] = feed_back.robots

                # 删除自修复的robot的event
                for r in need_repair_robots:
                    for index, event in enumerate(self.events):
                        if r == event.robot:  # todo robot.__eq__
                            del self.events[index]
                            # 因为每一个robot有且只有一个event，所以此处break
                            break

                # 构建新的robot协程，并更新记录和预激
                for r in need_repair_robots:
                    p_robot = physicalRobot(r, sim_time)
                    self.p_robots[robot.id] = p_robot
                    first_event = next(p_robot)
                    heapq.heappush(self.events, first_event)

                # 恢复MASys的自修复部分
                next(sim_sys)
        else:
            print(f"*** end of simulation time: {len(self.events)} events pending ***")


def main():
    pass


if __name__ == '__main__':
    main()
