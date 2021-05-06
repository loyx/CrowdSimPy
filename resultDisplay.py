import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.axes import Axes
from matplotlib.pyplot import MultipleLocator

from senseArea import SenseArea, Point
from MASys import MACrowdSystem
from robot import Robot


def pltSenseArea(ax, sense_area: SenseArea):
    ax.set_xlim(sense_area.startPoint[0], sense_area.endPoint[0])
    ax.set_ylim(sense_area.startPoint[1], sense_area.endPoint[1])


def pltRobotPath(ax: Axes, robot: Robot):
    colors = ['g', 'r', 'b', 'k']
    points = []
    for index, reg in enumerate(robot.planned_path):
        points.append(reg.represent_loc)
        if robot.C.move_mode == 'Land' and index < len(robot.planned_path)-1:
            points.append(Point(reg.represent_loc[0], robot.planned_path[index+1].represent_loc[1]))
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    return ax.plot(x, y, '-'+colors[robot.C.id], linewidth=0.5)


def pltMASys(ma_sys: MACrowdSystem):
    # style setting
    plt.figure(figsize=(10, 10), dpi=1000)
    mpl.rcParams['grid.linestyle'] = '-'
    ax: Axes
    fig, ax = plt.subplots()
    # ax.xaxis.set_major_locator(MultipleLocator(ma_sys.grid_granularity))
    # ax.yaxis.set_major_locator(MultipleLocator(ma_sys.grid_granularity))
    locator = MultipleLocator(ma_sys.grid_granularity)
    ax.xaxis.set_minor_locator(locator)
    ax.yaxis.set_minor_locator(locator)
    ax.xaxis.set_major_locator(MultipleLocator(2*ma_sys.grid_granularity))
    ax.yaxis.set_major_locator(MultipleLocator(2*ma_sys.grid_granularity))
    ax.set_aspect('equal')
    ax.grid(True, 'both', alpha=0.3)

    # plt sense area
    pltSenseArea(ax, ma_sys.sense_area)

    legend_line = []
    legend_label = []
    # plt tasks
    for task in ma_sys.tasks:
        for reg in task.TR:
            pos = reg.represent_loc
            line = ax.plot(pos[0], pos[1], 'k*', markersize=3)
            if not legend_line:
                legend_line.append(line[0])
                legend_label.append('tasks')
                # line.set_label('task')
    # ax.legend(['tasks'], loc=2, bbox_to_anchor=(1.05, 1), borderaxespad=0)

    styles = ['gx', 'r>', 'bo', 'kH']
    plted = [False] * len(styles)
    # plt robots
    for robot in ma_sys.robots:
        init_robot_loc = robot.init_reg.represent_loc
        ax.plot(init_robot_loc[0], init_robot_loc[1], styles[robot.C.id], markersize=3)
        line = pltRobotPath(ax, robot)
        if not plted[robot.C.id]:
            legend_line.append(line[0])
            legend_label.append(type(robot.C).__name__)
            # line.set_label(type(robot.C).__name__)
            plted[robot.C.id] = True

    ax.legend(legend_line, legend_label, loc=2, bbox_to_anchor=(1.03, 1), borderaxespad=0)
    # ax.legend(loc=2, bbox_to_anchor=(1.05, 1), borderaxespad=0)
    plt.show()
    # plt.savefig("ma_sys.png", dpi=1000)
