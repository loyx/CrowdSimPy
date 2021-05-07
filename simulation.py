import heapq
# import collections
from typing import Dict, List

from resultDisplay import pltMASys
from robot import Robot
from MASys import MACrowdSystem
from realWorld import RealWorld
from message import Message, FeedBack

# Event = collections.namedtuple("Event", "time robot action")


class Event:

    def __init__(self, time, robot, action):
        self.time = time
        self.robot = robot
        self.action = action

    def __repr__(self):
        return f"Event(time:{self.time}, robot:{self.robot}, action:{self.action})"

    def __iter__(self):
        return (i for i in (self.time, self.robot, self.action))

    def __lt__(self, other):
        return self.time < other.time


def physicalRobot(robot: Robot, start_time=0):
    assert robot.state == robot.idleState
    # robot 预激后，由MASys负责分配任务，并启动所有robot
    # 因而下一个yield时，robot的状态应该为MovingState
    time = yield Event(start_time, robot, "init")
    # 所有的robot都应从0开始计时，由Simulator估计moving时间，或者robot未被分配任务，处于IdleState

    assert robot.state == robot.movingState or robot.state == robot.idleState
    while not robot.isFinishMissions:  # todo 优化：brokenState
        # 因为robot的激活和规划由MASys完成，因而此循环里只有moving和sensing状态
        # 同时，我们应当假设idle，planing状态不花费时间

        time = yield Event(time, robot, "start sensing")
        assert robot.state == robot.sensingState
        # robot.sense()  # robot到到达目标地点，开始感知数据
        # 返回SensingState Robot，有Simulator估计Sensing时间

        time = yield Event(time, robot, "start moving")
        assert robot.state == robot.movingState or robot.state == robot.idleState
        # robot.submitTasks()  # robot完成感知提交任务
        # 返回MovingState Robot，由Simulator估计下一个Moving的时间，或完成所有任务处于Idle状态

        # robot 以 (移动-感知) 为一个操作周期，直至分配给其的任务执行完成


class Simulator:

    def __init__(self, p_robots, ma_sys, real_world):
        self.events: List[Event] = []
        self.p_robots: Dict = p_robots
        self.realWorld: RealWorld = real_world
        self.MASys: MACrowdSystem = ma_sys

    def run(self, end_time):
        print()
        print('-'*60, 'START SIMULATION', '-'*60)
        print("*** start event ***")
        # init
        # 预激robot
        for p_robot in self.p_robots.values():
            first_event = next(p_robot)
            # self.events.put(first_event)
            heapq.heappush(self.events, first_event)
        print("$$$ simulator: init p_robots $$$")

        # 预激MASys，并分配任务, 启动robot
        sim_sys = self.MASys.run()
        next(sim_sys)
        print("$$$ simulator: start MASys $$$")

        # start simulation
        sim_time = 0
        total_tasks = self.MASys.TaskNums
        finished = 0
        robot: Robot
        while sim_time < end_time:
            # if self.events.empty():
            if len(self.events) == 0:
                print("*** end of events ***")
                # plt
                plt = pltMASys(self.MASys, False, False)
                try:
                    next(plt)
                except StopIteration:
                    pass
                break

            # sim_time, robot, action = self.events.get()
            sim_time, robot, action = heapq.heappop(self.events)
            # print("time:", sim_time, robot.id * '  ', robot, action, end=' ')
            print(f"time:{sim_time:10.3f} | {robot}: {action:13}", end=" | ")

            # 这些操作发生在状态转化的那个瞬间  # todo 优化：brokenState
            if action == "init":
                print("start moving")
                feed_back = FeedBack(0)
            elif robot.state == robot.movingState:
                if self.realWorld.canSense(robot):
                    print("can sense this reg")
                    robot.sense(sim_time)
                    feed_back = FeedBack(0)
                else:
                    print("cannot sense this reg!")
                    message = Message(3, robot.id, robot, robot.current_region, sim_time)
                    feed_back: FeedBack = sim_sys.send(message)
            elif robot.state == robot.sensingState:
                submit_tasks = ['Task'+str(t.id) for t in robot.currentTasks]
                finished += len(submit_tasks)
                print(f"robot submitTask: reg{robot.current_region.id}, {submit_tasks}, {finished}/{total_tasks}")
                robot.submitTasks(sim_time)
                if robot.canFinishTaskInTime(sim_time):
                    message = Message(0, robot.id, robot, robot.current_region, sim_time)
                else:
                    message = Message(2, robot.id, robot, robot.current_region, sim_time)
                feed_back: FeedBack = sim_sys.send(message)
            else:
                raise RuntimeError("error robot")

            if feed_back.status_code == 0:
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
                    # 当robot处于sensingState时，证明这是一次热自修复，不需要删除events
                    if r.state == r.sensingState:
                        continue
                    for index, event in enumerate(self.events):
                        # 这里必须使用id判断相等，因为一个robot被多个对象引用
                        if r == event.robot:
                            del self.events[index]
                            # 因为每一个robot有且只有一个event，所以此处break
                            # 注意一般不能边迭代边del，这里是特殊情况
                            # todo 优化：更优的方式是将其赋值为一个nonsense event
                            break

                # 构建新的robot协程，并更新记录和预激
                for r in need_repair_robots:
                    # 当robot处于sensingState时，证明这是一次热自修复，不需要重建协程
                    if r.state == r.sensingState:
                        continue
                    p_robot = physicalRobot(r, sim_time)
                    self.p_robots[r.id] = p_robot
                    first_event = next(p_robot)
                    heapq.heappush(self.events, first_event)

                # 恢复MASys的自修复部分
                next(sim_sys)

        else:
            print(f"*** end of simulation time: {len(self.events)} events pending ***")


if __name__ == '__main__':
    pass
