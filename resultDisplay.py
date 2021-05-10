import itertools
import time

import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import colors, cm
from matplotlib.axes import Axes
from matplotlib.colors import ListedColormap
from matplotlib.pyplot import MultipleLocator
import numpy as np

from senseArea import Point
from robot import Robot

SAVE = True


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


def pltMASys(ma_sys, async_use=False, save=SAVE):
    save = SAVE
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
        plt.savefig(f"ma_sys_{time.time()}.png", dpi=1000)
    plt.show()


def pltSenseMap(sense_map, save=SAVE):
    plt.figure(figsize=(10, 10), dpi=1000)
    fig, ax = plt.subplots()
    ax: Axes
    ax.set_aspect('equal')
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))
    ax.grid()

    x = np.arange(sense_map.grid_size[0])
    y = np.arange(sense_map.grid_size[1])
    z = []
    for i in range(sense_map.size[0]):
        sum_p = sum(sense_map[i, j, 0][0] for j in range(sense_map.size[1]))
        z.append(sum_p)
    mx, my = np.meshgrid(x, y)
    mz = np.array(z).reshape(mx.shape)

    # pc = ax.pcolormesh(mx, my, mz, shading='auto')
    greens = cm.get_cmap('Greens', 256)(np.linspace(0, 1, 127))
    white = np.array([1, 1, 1, 1])
    # greens[:10, :] = white
    oranges = cm.get_cmap('Oranges', 256)(np.linspace(0, 1, 128))
    # oranges[:10, :] = white
    new_colors = np.array([c for c in itertools.chain([white], reversed(oranges), greens)])
    new_cmp = ListedColormap(new_colors)

    min_z, max_z = mz.min(), mz.max()
    z_padding = (max_z - min_z) / 4
    pc = ax.contourf(mx, my, mz, cmap=new_cmp, levels=256,
                     norm=colors.CenteredNorm(),
                     vmin=min_z - z_padding, vmax=mz.max() + z_padding)
    # pc = ax.contourf(mx, my, mz, cmap=new_cmp, levels=256, vmin=0, vmax=mz.max())
    # pc = ax.pcolormesh(mx, my, mz, cmap=new_cmp, shading='auto', norm=colors.CenteredNorm())
    fig.colorbar(pc, ax=ax)
    if save:
        plt.savefig(f"senseMap_{time.time()}.png", dpi=1000)
    plt.show()
