import matplotlib.pyplot as plt
import numpy as np
# plt.rcParams['font.sans-serif'] = ['SimHei']
# plt.rcParams['axes.unicode_minus'] = False

cov = [1, 0.95, 1, 0.9]
dis = [42227.42407, 42601.32056, 116196.5311, 102025.9637]

covs = [
    [1, 0.95, 1, 0.975],
    [0.99, 0.95, 0.985, 0.9, ],
    [0.9875, 0.825, 0.92, 0.8375, ]
]
diss = [
    [32112.09643, 30118.12386, 77841.29649, 64240.15626],
    [42227.42407, 42601.32056, 116196.5311, 102025.9637, ],
    [63380.77139, 60734.86319, 162134.1757, 132181.9752]
]
diss = [[x/100 for x in y] for y in diss]

x = [40, 60, 80]
# x = [1, 2, 3]
w = 3


def pltbar(i, label, width, c):
    # w1 = [value[i] for value in covs]
    w1 = [value[i] for value in diss]
    plt.bar(np.array(x)+width, w1, label=label, width=w, color=c)


# plt.ylim((0.75, 1.1))
# plt.ylim((0.75, 1.1))
labels = ['Greedy+self_repair', 'Greedy', 'Random+self_repair', 'Random']
# colors = ['r', 'b', 'g', 'k']
# colors = ['pink', 'greenyellow', 'olive', 'skyblue']
colors = plt.get_cmap('Paired').colors
# colors = plt.get_cmap('Set2').colors
widths = [-w*3/2, -w/2, w/2, w*3/2]
for i in range(4):
    pltbar(i, labels[i], widths[i], colors[i])

plt.xlabel("Number of tasks")
# plt.ylabel("Task cov (%)")
plt.ylabel("Participant movement distance (km)")
plt.legend(ncol=2, )
# plt.savefig("exp2_cov.png", dpi=1000, bbox_inches='tight')
plt.savefig("exp2_dis.png", dpi=1000, bbox_inches='tight')
plt.show()
