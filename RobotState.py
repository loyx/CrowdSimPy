from abc import ABC


class StateError(Exception):
    """
    robot state transition error
    """


class RobotState(ABC):

    def __init__(self, robot):
        self.robot = robot

    def __repr__(self):
        return f"RobotState: {type(self).__name__[:-5]}"

    def assignTask(self, reg, task, used_sensor):
        raise StateError(f"{type(self).__name__} cannot assignTask()")

    def cancelPlan(self, time, regions):
        raise StateError(f"{type(self).__name__} cannot cancelPlan()")

    def executeMissions(self):
        raise StateError(f"{type(self).__name__} cannot executeMissions()")

    def submitTask(self, time):
        raise StateError(f"{type(self).__name__} cannot submitTask()")

    def sense(self):
        raise StateError(f"{type(self).__name__} cannot sense()")

    def broken(self):
        raise StateError(f"{type(self).__name__} cannot broken()")

    def assignTaskOpr(self, reg, task, used_sensor, ideal_time):  # todo 优化：函数形式
        # update task and sensor record
        self.robot.task_in_reg.append([task])
        self.robot.sensor_in_reg.append([used_sensor])

        # update path record
        self.robot.planned_distance.append(self.robot.planned_distance[-1] + self.robot.taskDistance(reg))
        self.robot.planned_path.append(reg)

        # update time record
        time_used = ideal_time - self.robot.finish_time[-1]
        self.robot.ideal_time_used.append(time_used)
        self.robot.finish_time.append(ideal_time)
        sensing_time = self.robot.C.intraD(reg) / self.robot.C.v
        self.robot.ideal_sensing_time.append(sensing_time)
        self.robot.ideal_moving_time.append(time_used - sensing_time)


class IdleState(RobotState):

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
        current_cursor = self.robot.current_cursor
        start_reg = self.robot.planned_path[current_cursor-1]
        end_reg = self.robot.current_task_region
        assert end_reg == self.robot.planned_path[current_cursor]
        percentage = (time-self.robot.finish_time[current_cursor-1]) / self.robot.ideal_time_used[current_cursor]
        self.robot.current_region = self.robot.C.getLocation(start_reg, end_reg, percentage, regions)
        self.robot.location = self.robot.current_region.randomLoc()

        # clear plan
        self.robot.clearRecord(self.robot.current_cursor)
        self.robot.current_cursor -= 1
        self.robot.current_task_region = None

        # change state
        self.robot.state = self.robot.idleState

    def sense(self):
        self.robot.current_region = self.robot.current_task_region
        self.robot.location = self.robot.current_region.randomLoc()
        self.robot.state = self.robot.sensingState


class SensingState(RobotState):

    def assignTask(self, reg, task, used_sensor):
        ideal_time = self.robot.idealFinishTime(reg, used_sensor)

        # 不同于idleState的分配任务，SensingState只能在之后分配任务，不能并发执行
        self.assignTaskOpr(reg, task, used_sensor, ideal_time)

        # self.robot.state = self.robot.sensingState

    def cancelPlan(self, time, regions):
        assert self.robot.current_region == self.robot.current_task_region
        self.robot.location = self.robot.current_region.randomLoc()
        self.robot.clearRecord(self.robot.current_cursor + 1)

        # self.robot.state = self.robot.sensingState

    def submitTask(self, time):
        self.robot.finish_time[self.robot.current_cursor] = time  # 更新real time
        self.robot.current_cursor += 1
        if self.robot.current_cursor < len(self.robot.planned_path):
            self.robot.current_task_region = self.robot.planned_path[self.robot.current_cursor]
            self.robot.state = self.robot.movingState
        else:
            self.robot.state = self.robot.idleState


class BrokenState(RobotState):
    pass
