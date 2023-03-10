import itertools
import time
from typing import List

import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import colors, cm
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.colors import ListedColormap
from matplotlib.pyplot import MultipleLocator
import numpy as np
from scipy import interpolate

from senseArea import Point
from robot import Robot

SAVE = True
PRINT_INIT = False
PRINT_PATH_STYLE = False
PRINT_ALGO_STYLE = False
PRINT_SENSE_MAP = False
PRINT_REPAIR = True

# mpl.style.use('seaborn')
GLOBAL_COLOR = ['royalblue', 'purple', 'tab:red', 'dodgerblue']


def workerPath(p1, mid, p2):
    if p1 == p2:
        return []
    nodes = np.array([list(p1), list(mid), list(p2)])
    x = nodes[:, 0]
    y = nodes[:, 1]
    tck, u, *_ = interpolate.splprep([x, y], k=2)
    xnew, ynew = interpolate.splev(np.linspace(0, 1, 10), tck, der=0)
    return [Point(a, b) for a, b in zip(xnew, ynew)][1:-1]


def pltRobotPath(ax: Axes, robot: Robot, alpha=False):
    path_colors = GLOBAL_COLOR
    points = []
    for index, reg in enumerate(robot.planned_path):
        points.append(reg.represent_loc)
        if robot.C.move_mode == 'Land' and index < len(robot.planned_path) - 1:
            p1 = robot.planned_path[index + 1].represent_loc
            p2 = reg.represent_loc
            mid = Point(p1[0], p2[1])
            if robot.C.id == 1:
                points.append(mid)
            else:
                mid = Point((p1[0] + p2[0]) / 2, p2[1] if index % 2 else p1[1])
                points.extend(workerPath(p2, mid, p1))
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    if alpha:
        return ax.plot(x, y, '--', color=path_colors[robot.C.id], linewidth=0.5, alpha=0.5)
    else:
        return ax.plot(x, y, '-', color=path_colors[robot.C.id], linewidth=0.8)


def pltMASys(ma_sys, async_use=False, save=SAVE):
    save = SAVE
    # style setting
    # plt.figure(figsize=(10, 10), dpi=1000)
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
    ax.tick_params('x', rotation=15)
    ax.grid(True, 'both', alpha=0.3)

    # plt sense area
    xlen, ylen = ma_sys.sense_area.len
    ax.set_xlim(-ma_sys.grid_granularity, xlen + ma_sys.grid_granularity)
    ax.set_ylim(-ma_sys.grid_granularity, ylen + ma_sys.grid_granularity)

    legend_line = []
    legend_label = []

    styles = ['x', '>', 'o', 'H']
    pc = GLOBAL_COLOR
    plted = [False] * len(styles)

    if async_use:
        for robot in ma_sys.robots:
            pltRobotPath(ax, robot, True)
        yield

    # plt robots
    check = []
    for robot in ma_sys.robots:
        if PRINT_PATH_STYLE:
            if robot.C.id == 2:
                continue
        init_robot_loc = robot.current_region.represent_loc
        p = ax.plot(init_robot_loc[0], init_robot_loc[1], styles[robot.C.id],
                    color=pc[robot.C.id], markersize=3 if not PRINT_INIT else 6)
        if PRINT_INIT:
            if robot.C.id not in check:
                check.append(robot.C.id)
                legend_line.append(p[0])
                legend_label.append(type(robot.C).__name__)
            continue
        line = pltRobotPath(ax, robot)
        if not plted[robot.C.id] and line:
            legend_line.append(line[0])
            legend_label.append(type(robot.C).__name__)
            plted[robot.C.id] = True

    flag = True
    # plt tasks
    for task in ma_sys.tasks:
        for reg in task.TR:
            pos = reg.represent_loc
            line = ax.plot(pos[0], pos[1], 'ko', markersize=1 if not PRINT_INIT else 2)
            if flag:
                flag = False
                legend_line.append(line[0])
                legend_label.append('tasks')

    # plt legend
    # ax.legend(legend_line, legend_label, loc=2, bbox_to_anchor=(1.03, 1), borderaxespad=0)
    ax.legend(legend_line, legend_label, ncol=4, bbox_to_anchor=(1, 1.075), borderaxespad=0)

    if PRINT_INIT or PRINT_PATH_STYLE or PRINT_ALGO_STYLE:
        plt.savefig(f"ma_sys_{time.time()}.png", dpi=1000, bbox_inches='tight')
        plt.show()
        raise RuntimeError("stop!")

    # show
    if save:
        plt.savefig(f"ma_sys_{time.time()}.png", dpi=1000, bbox_inches='tight')
    plt.show()


def pltSenseMap(sense_map, save=SAVE):
    if PRINT_REPAIR:
        return
    plt.figure(figsize=(10, 10), dpi=1000)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax: Axes
    # ax.set_aspect('equal')
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))
    ax.tick_params('x', labelcolor='w')
    ax.tick_params('y', labelcolor='w')
    # ax.grid()

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
    # pc = ax.contourf(mx, my, mz, cmap=new_cmp, levels=256,
    #                  norm=colors.CenteredNorm(),
    #                  vmin=min_z - z_padding, vmax=mz.max() + z_padding)
    # pc = ax.contourf(mx, my, mz, cmap=new_cmp, levels=256, vmin=0, vmax=mz.max())
    # pc = ax.pcolormesh(mx, my, mz, cmap=new_cmp, shading='auto', norm=colors.CenteredNorm())
    # fig.colorbar(pc, ax=ax)
    # surf = ax.plot_surface(mx, my, mz, cmap=new_cmp)
    surf = ax.plot_wireframe(mx, my, mz)

    if PRINT_SENSE_MAP:
        # plt.savefig(f"senseMap_{time.time()}.png", dpi=1000)
        plt.show()
        raise RuntimeError('stop!')

    if save:
        plt.savefig(f"senseMap_{time.time()}.png", dpi=1000)
    plt.show()
