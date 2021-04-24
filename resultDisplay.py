import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.axes import Axes
from matplotlib.pyplot import MultipleLocator

from senseArea import SenseArea
from MASys import MACrowdSystem
from robot import Robot


def pltSenseArea(ax, sense_area: SenseArea):
    ax.set_xlim(sense_area.startPoint[0], sense_area.endPoint[0])
    ax.set_ylim(sense_area.startPoint[1], sense_area.endPoint[1])


def pltRobotPath(ax: Axes, robot: Robot):
    colors = ['g', 'y', 'b', 'k']
    points = [reg.randomLoc() for reg in robot.planned_path]
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    ax.plot(x, y, '-'+colors[robot.C.id], linewidth=2)


def pltMASys(ma_sys: MACrowdSystem):
    # style setting
    plt.figure(figsize=(10, 10), dpi=1000)
    mpl.rcParams['grid.linestyle'] = '--'
    ax: Axes
    fig, ax = plt.subplots()
    # ax.xaxis.set_major_locator(MultipleLocator(ma_sys.grid_granularity))
    # ax.yaxis.set_major_locator(MultipleLocator(ma_sys.grid_granularity))
    locator = MultipleLocator(ma_sys.grid_granularity)
    ax.xaxis.set_minor_locator(locator)
    ax.yaxis.set_minor_locator(locator)
    ax.set_aspect('equal')
    ax.grid(True, 'both', alpha=0.7)

    # plt sense area
    pltSenseArea(ax, ma_sys.sense_area)

    # plt tasks
    for task in ma_sys.tasks:
        for reg in task.TR:
            pos = reg.randomLoc()
            ax.plot(pos[0], pos[1], 'r*', markersize=3)

    styles = ['gx', 'y>', 'bo', 'kH']
    # plt robots
    for robot in ma_sys.robots:
        # print(robot.location)
        ax.plot(robot.location[0], robot.location[1], styles[robot.C.id], markersize=3)
        pltRobotPath(ax, robot)

    plt.show(dpi=500)
    # plt.savefig("ma_sys.png", dpi=1000)
