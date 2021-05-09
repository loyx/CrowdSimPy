import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import colors, cm
from matplotlib.axes import Axes
from matplotlib.colors import ListedColormap
from matplotlib.pyplot import MultipleLocator
import numpy as np

from senseArea import Point
from robot import Robot


def pltRobotPath(ax: Axes, robot: Robot, alpha=False):
    path_colors = ['g', 'r', 'b', 'k']
    points = []
    for index, reg in enumerate(robot.planned_path):
        points.append(reg.represent_loc)
        if robot.C.move_mode == 'Land' and index < len(robot.planned_path) - 1:
            points.append(Point(reg.represent_loc[0], robot.planned_path[index + 1].represent_loc[1]))
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    if alpha:
        return ax.plot(x, y, '--' + path_colors[robot.C.id], linewidth=0.5, alpha=0.5)
    else:
        return ax.plot(x, y, '-' + path_colors[robot.C.id], linewidth=0.5)


def pltMASys(ma_sys, async_use=False, save=False):
    # style setting
    plt.figure(figsize=(10, 10), dpi=1000)
    mpl.rcParams['grid.linestyle'] = '-'
    ax: Axes
    fig, ax = plt.subplots()

    # plt grid
    locator = MultipleLocator(ma_sys.grid_granularity)
    ax.xaxis.set_minor_locator(locator)
    ax.yaxis.set_minor_locator(locator)
    ax.xaxis.set_major_locator(MultipleLocator(2 * ma_sys.grid_granularity))
    ax.yaxis.set_major_locator(MultipleLocator(2 * ma_sys.grid_granularity))
    ax.set_aspect('equal')
    ax.grid(True, 'both', alpha=0.3)

    # plt sense area
    xlen, ylen = ma_sys.sense_area.len
    ax.set_xlim(-ma_sys.grid_granularity, xlen + ma_sys.grid_granularity)
    ax.set_ylim(-ma_sys.grid_granularity, ylen + ma_sys.grid_granularity)

    legend_line = []
    legend_label = []

    styles = ['gx', 'r>', 'bo', 'kH']
    plted = [False] * len(styles)

    if async_use:
        for robot in ma_sys.robots:
            pltRobotPath(ax, robot, True)
        yield

    # plt robots
    for robot in ma_sys.robots:
        init_robot_loc = robot.current_region.represent_loc
        ax.plot(init_robot_loc[0], init_robot_loc[1], styles[robot.C.id], markersize=3)
        line = pltRobotPath(ax, robot)
        if not plted[robot.C.id]:
            legend_line.append(line[0])
            legend_label.append(type(robot.C).__name__)
            plted[robot.C.id] = True

    # plt tasks
    for task in ma_sys.tasks:
        for reg in task.TR:
            pos = reg.represent_loc
            line = ax.plot(pos[0], pos[1], 'ks', markersize=1)
            if not legend_line:
                legend_line.append(line[0])
                legend_label.append('tasks')

    # plt legend
    ax.legend(legend_line, legend_label, loc=2, bbox_to_anchor=(1.03, 1), borderaxespad=0)

    # show
    if save:
        plt.savefig("ma_sys.png", dpi=1000)
    plt.show()


# def pltSenseMap(sense_map: SenseMap):
def pltSenseMap(sense_map):
    plt.figure(figsize=(10, 10), dpi=1000)
    fig, ax = plt.subplots()
    ax: Axes
    ax.set_aspect('equal')
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))

    x = np.arange(sense_map.grid_size[0])
    y = np.arange(sense_map.grid_size[1])
    z = []
    for i in range(sense_map.size[0]):
        sum_p = sum(sense_map[i, j, 0][0] for j in range(sense_map.size[1]))
        z.append(sum_p)
    mx, my = np.meshgrid(x, y)
    mz = np.array(z).reshape(mx.shape)

    # pc = ax.pcolormesh(mx, my, mz, shading='auto')
    new_colors = cm.get_cmap('Greens', 256)(np.linspace(0, 1, 256))
    white = np.array([1, 1, 1, 1])
    new_colors[:20, :] = white
    new_cmp = ListedColormap(new_colors)
    pc = ax.contourf(mx, my, mz, cmap=new_cmp, vmin=mz.min(), vmax=mz.max() * 1.5)
    fig.colorbar(pc, ax=ax)
    plt.show()
