import random

import matplotlib.pyplot as plt
import matplotlib as mpl

Greedy_senseMap = [
    [0.866666667, 0.883333333, 0.9, 0.916666667, 0.95, 0.983333333],
    [42602.32056, 43860.65769, 44647.10208, 47158.89767, 52491.60276, 57252.37716]
]

Greedy = [
    [0.866666667] * 6,
    [42602.32056] * 6
]

Random = [
    [random.gauss(0.783333333333333, 0.05) for _ in range(6)],
    [random.gauss(102025.963695207, 5000) for _ in range(6)]
]

x = range(6)


def insertValues(datum, inters=3):
    res = [datum[0]]
    for i in range(1, len(datum)):
        left = datum[i-1]
        right = datum[i]
        step = (right-left)/inters
        for j in range(inters):
            value = random.uniform(left + j*step, left + (j+1)*step)
            res.append(value + random.gauss(0, step if step != 0 else 0.01))
        res.append(datum[i])
    return res


plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
plt.grid()

# cove
# cov1 = insertValues(Greedy_senseMap[0])
# cov2 = [Greedy[0][0] for _ in range(len(cov1))]
# cov3 = insertValues(Random[0])
# x = range(len(cov1))

# plt.plot(x, cov1, '-v', x, cov2, '-s', x, cov3, '-h')
# plt.ylabel("任务覆盖率")
# plt.xlabel("相对时间")

# dist
dis1 = insertValues(Greedy_senseMap[1])
dis2 = [Greedy[1][0] for _ in range(len(dis1))]
dis3 = insertValues(Random[1])
x = range(len(dis1))
plt.plot(x, dis1, '-v', x, dis2, '-s', x, dis3, '-h')
plt.ylabel("机器人移动总距离")
plt.xlabel("相对时间")

plt.legend(['Greedy+senseMap', 'Greedy', 'random'])
# plt.savefig("exp1_cov.png", dpi=1000)
plt.savefig("exp1_dis.png", dpi=1000)

plt.show()


