from abc import ABC

from robot import Robot


class StateError(Exception):
    """
    robot state transition error
    """


class RobotState(ABC):

    def __init__(self, robot):
        self.robot: Robot = robot

    def __repr__(self):
        return f"RobotState: {type(self).__name__[:-5]}"

    def assignTask(self, reg, task, used_sensor):
        raise StateError(f"{type(self).__name__} cannot assignTask()")

    def cancelPlan(self):
        raise StateError(f"{type(self).__name__} cannot cancelPlan()")

    def executeMissions(self):
        raise StateError(f"{type(self).__name__} cannot executeMissions()")

    def submitTask(self):
        raise StateError(f"{type(self).__name__} cannot submitTask()")

    def sense(self):
        raise StateError(f"{type(self).__name__} cannot sense()")

    def broken(self):
        raise StateError(f"{type(self).__name__} cannot broken()")


class IdleState(RobotState):

    def assignTask(self, reg, task, used_sensor):
        ideal_time = self.robot.idealFinishTime(reg, used_sensor)

        # 如果理想完成时间与之前相同，说明该任务与前一任务并发执行
        if ideal_time == self.robot.ideal_finish_time[-1]:
            self.robot.task_in_reg[-1].append(task)
            self.robot.sensor_in_reg[-1].append(used_sensor)
        else:
            self.robot.task_in_reg.append([task])
            self.robot.sensor_in_reg.append([used_sensor])
            self.robot.ideal_finish_time.append(ideal_time)

        # self.robot.state = self.robot.idleState

    def executeMissions(self):
        self.robot.current_cursor += 1
        self.robot.current_task_region = self.robot.planned_path[self.robot.current_cursor]
        self.robot.state = self.robot.movingState


class MovingState(RobotState):

    def cancelPlan(self):
        self.robot.clearRecord(self.robot.current_cursor)
        self.robot.current_cursor -= 1
        self.robot.current_task_region = None

        self.robot.state = self.robot.idleState

    def sense(self):
        self.robot.state = self.robot.sensingState


class SensingState(RobotState):

    def assignTask(self, reg, task, used_sensor):
        ideal_time = self.robot.idealFinishTime(reg, used_sensor)

        # 不同于idleState的分配任务，SensingState只能在之后分配任务，不能并发执行
        self.robot.task_in_reg.append([task])
        self.robot.sensor_in_reg.append([used_sensor])
        self.robot.ideal_finish_time.append(ideal_time)

        # self.robot.state = self.robot.sensingState

    def cancelPlan(self):
        self.robot.clearRecord(self.robot.current_cursor + 1)

        # self.robot.state = self.robot.sensingState

    def submitTask(self):
        self.robot.current_cursor += 1
        if self.robot.current_cursor < len(self.robot.planned_path):
            self.robot.current_task_region = self.robot.planned_path[self.robot.current_cursor]
            self.robot.state = self.robot.movingState
        else:
            self.robot.state = self.robot.idleState


class BrokenState(RobotState):
    pass
