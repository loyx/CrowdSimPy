from abc import ABC


class StateError(Exception):
    """
    robot state transition error
    """


class RobotState(ABC):

    def __init__(self, robot):
        self.robot = robot

    def __repr__(self):
        return f"RobotState: {type(self).__name__[:-5]:>7}"

    def assignTask(self, reg, task, used_sensor):
        raise StateError(f"{type(self).__name__} cannot assignTask()")

    def cancelPlan(self, time, regions):
        raise StateError(f"{type(self).__name__} cannot cancelPlan()")

    def executeMissions(self):
        raise StateError(f"{type(self).__name__} cannot executeMissions()")

    def submitTask(self, time):
        raise StateError(f"{type(self).__name__} cannot submitTask()")

    def sense(self, time):
        raise StateError(f"{type(self).__name__} cannot sense()")

    def skipSense(self, time):
        raise StateError(f"{type(self).__name__} cannot skipSense()")

    def broken(self):
        raise StateError(f"{type(self).__name__} cannot broken()")

    def assignTaskOpr(self, reg, task, used_sensor, ideal_time):  # todo 优化：函数形式
        # update task and sensor record
        robot = self.robot
        robot.task_in_reg.append([task])
        robot.sensor_in_reg.append([used_sensor])

        # update path record
        robot.planned_distance.append(robot.planned_distance[-1] + robot.taskDistance(reg))
        robot.planned_path.append(reg)

        # update time record
        time_used = ideal_time - robot.finish_time[-1]
        robot.ideal_time_used.append(time_used)
        robot.finish_time.append(ideal_time)
        sensing_time = robot.C.intraD(reg) / robot.C.v
        robot.ideal_sensing_time.append(sensing_time)
        robot.ideal_moving_time.append(time_used - sensing_time)


class IdleState(RobotState):

    def cancelPlan(self, time, regions):
        # 在IdleState取消计划，则对于已完成任务的机器人应该将current_cursor恢复成类似初始状态的形式
        # todo 重构：这样的设计非常不好
        if self.robot.current_cursor > 0:
            self.robot.current_cursor -= 1
        pass

    def assignTask(self, reg, task, used_sensor):
        ideal_time = self.robot.idealFinishTime(reg, used_sensor, task)

        # 如果理想完成时间与之前相同，说明该任务与前一任务并发执行
        if ideal_time == self.robot.finish_time[-1]:
            self.robot.task_in_reg[-1].append(task)
            self.robot.sensor_in_reg[-1].append(used_sensor)
        else:
            self.assignTaskOpr(reg, task, used_sensor, ideal_time)

        # self.robot.state = self.robot.idleState

    def executeMissions(self):
        self.robot.current_cursor += 1
        if self.robot.isFinishMissions:  # 当机器人未被分配任何任务时，触发此种情况
            return
        self.robot.current_task_region = self.robot.planned_path[self.robot.current_cursor]
        self.robot.state = self.robot.movingState


class MovingState(RobotState):

    def cancelPlan(self, time, regions):
        # update location
        robot = self.robot
        current_cursor = robot.current_cursor
        start_reg = robot.planned_path[current_cursor - 1]
        end_reg = robot.current_task_region
        assert end_reg == robot.planned_path[current_cursor]
        # todo 优化：实际时间一般都比理论用时长，因此此处估计的已行进距离会比实际多一点
        percentage = (time - robot.finish_time[current_cursor - 1]) / robot.ideal_time_used[current_cursor]
        robot.current_region = robot.C.getLocation(start_reg, end_reg, percentage, regions)
        robot.location = robot.current_region.randomLoc()

        # clear plan
        robot.clearRecord(robot.current_cursor)

        # 更新相关record
        # 当处于movingState的robot cancelPlan() 时，根据当前reg建立新起点
        # 各record应和__init__中类似
        robot.planned_path.append(robot.current_region)
        robot.finish_time.append(time)  # 起点看为已完成任务，则finish time为当前时间
        robot.task_in_reg.append([None])
        robot.sensor_in_reg.append([None])
        robot.ideal_time_used.append(0)
        robot.ideal_moving_time.append(0)
        robot.ideal_sensing_time.append(0)
        # 需要更新计划距离
        dis = robot.planned_distance[-1] + robot.C.interD(robot.planned_path[-2], robot.planned_path[-1])
        robot.planned_distance.append(dis)

        # 此时current_cursor不为0，但相当于初始状态
        robot.current_task_region = None

        # change state
        robot.state = robot.idleState

    def sense(self, time):

        # 更新robot位置
        self.robot.current_region = self.robot.current_task_region
        self.robot.location = self.robot.current_region.randomLoc()

        # begin tasks transaction
        for task in self.robot.currentTasks:
            task.beginSubTaskTransaction(self.robot.current_region, time)

        # change state
        self.robot.state = self.robot.sensingState

    def skipSense(self, time):
        robot = self.robot

        # 更新robot位置
        robot.current_region = robot.current_task_region
        robot.location = robot.current_region.randomLoc()

        # no begin tasks transaction !
        # self.robot.state = self.robot.sensingState

        # 跳过当前任务
        # no commit task transaction !
        # 但要更新real time
        robot.finish_time[robot.current_cursor] = time

        # 跳到下一任务
        robot.current_cursor += 1
        if robot.current_cursor < len(robot.planned_path):
            robot.current_task_region = robot.planned_path[robot.current_cursor]
            # robot.state = robot.movingState
        else:
            robot.state = robot.idleState


class SensingState(RobotState):

    def executeMissions(self):
        # do not thing but need it
        pass

    def assignTask(self, reg, task, used_sensor):
        ideal_time = self.robot.idealFinishTime(reg, used_sensor, task)

        # 不同于idleState的分配任务，SensingState只能在之后分配任务，不能并发执行
        self.assignTaskOpr(reg, task, used_sensor, ideal_time)

        # self.robot.state = self.robot.sensingState

    def cancelPlan(self, time, regions):
        assert self.robot.current_region == self.robot.current_task_region
        self.robot.location = self.robot.current_region.randomLoc()
        self.robot.clearRecord(self.robot.current_cursor + 1)

        # self.robot.state = self.robot.sensingState

    def submitTask(self, time):
        robot = self.robot

        # commit task transaction
        for task in robot.currentTasks:
            task.commitSubTaskTransaction(robot.current_task_region, time)

        # 更新real time
        robot.finish_time[robot.current_cursor] = time

        # 切换下一任务
        robot.current_cursor += 1
        if robot.current_cursor < len(robot.planned_path):
            robot.current_task_region = robot.planned_path[robot.current_cursor]
            robot.state = robot.movingState
        else:
            robot.state = robot.idleState


class BrokenState(RobotState):
    pass
