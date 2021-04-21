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

    def assignTask(self):
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

    def assignTask(self):
        self.robot.state = self.robot.planingState


class PlaningState(RobotState):

    def assignTask(self):
        # self.robot.state = self.robot.idleState
        pass

    def executeMissions(self):
        self.robot.state = self.robot.movingState


class MovingState(RobotState):

    def cancelPlan(self):
        self.robot.state = self.robot.idleState

    def sense(self):
        self.robot.state = self.robot.sensingState


class SensingState(RobotState):

    def submitTask(self):
        if self.robot.current_cursor < len(self.robot.planned_path):
            self.robot.state = self.robot.movingState
        else:
            self.robot.state = self.robot.idleState


class BrokenState(RobotState):
    pass
