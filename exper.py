import random

from MASys import MACrowdSystem, GreedyBaseAlgor
from senseArea import SenseArea, Point
from task import TimeRange, Task
from concreteRobot import UAV, UV, SmallUV
from sensor import Sensor
from robot import Robot
from simulation import Simulator, physicalRobot
from realWorld import RealWorld
from resultDisplay import pltMASys

# experiment parameters
RANDOM_SEED = None
UAV_NUMS = 5
UV_NUMS = 8
SMALL_UV_NUMS = 12
TASK_NUMS = 30

area_len = 100

# experiment setting
random.seed(RANDOM_SEED)

# senseArea
print("*** System sense area ***")
start_point = Point(0, 0)
end_point = Point(area_len, area_len)
sense_area = SenseArea(start_point, end_point)
grid_granularity = 5
print(sense_area)
print()

# system time range
print('*** system time range ***')

time_long = 86400
sys_time_range = TimeRange(0, time_long)
time_granularity = 3600
print(sys_time_range)
print()

# sensors
print("*** System Sensors ***")
camera = Sensor(0, "camera", 1080, 'p', 20, 'm')
baroceptor = Sensor(1, "baroceptor", 1000, 'kPa', 10, 'mm')
temperature = Sensor(2, "temperature", 80, 'C', 1, 'm')
combustible = Sensor(3, "combustible", 4, '%LEL', 5, 'm')
PM25 = Sensor(4, "PM25", 5, '%LEL', 6, 'm')
toxic = Sensor(5, "toxic", 5, '%LEL', 10, 'm')
sensors = [camera, baroceptor, temperature, combustible, PM25, toxic]
for s in sensors:
    print(s)
print()

# robots category
print("*** System Robot categories ***")
# uav
# uav_sensor = [camera, baroceptor, temperature, PM25]
uav_sensor = sensors
uav_physical_parameters = {'length': 250, 'width': 180, 'height': 70, 'weight': 0.4}
uav_category = UAV(0, "uav", uav_sensor, 16.11, uav_physical_parameters)
print(uav_category)

# uv
# uv_sensor = [camera, baroceptor, temperature, PM25, ]
uv_sensor = sensors
uv_physical_parameters = {'length': 4000, 'width': 1700, 'height': 1400, 'weight': 1900}
uv_category = UV(1, "uv", uv_sensor, 11.11, uv_physical_parameters)
print(uv_category)

# SmallUV
small_uv_sensor = sensors
small_uv_physical_parameters = {'length': 2000, 'width': 850, 'height': 700, 'weight': 900}
small_uv_category = SmallUV(2, "small uv", small_uv_sensor, 10, small_uv_physical_parameters)
print(small_uv_category)
print()

# task allocation algorithm
algorithm = GreedyBaseAlgor()

# MASys
multi_agents_system = MACrowdSystem(
    sense_area,
    grid_granularity,
    sys_time_range,
    time_granularity,
    [uav_category, uv_category, small_uv_category],
    algorithm,
)
print("*** senseMap ***")
print(multi_agents_system.senseMap)
print()

print("*** sense Regions ***")
Regions = multi_agents_system.Regions
print(Regions)
print()

# robots
print("*** Robots ***")
r_id_cnt = -1
# uav robots
uav_robots = []
for i in range(UAV_NUMS):
    uav_robots.append(Robot(r_id_cnt := r_id_cnt + 1, uav_category, Regions[random.randrange(0, len(Regions))]))
print("--- uav robots ---")
print(uav_robots)

# uv robots
uv_robots = []
for i in range(UV_NUMS):
    uv_robots.append(Robot(r_id_cnt := r_id_cnt + 1, uv_category, Regions[random.randrange(0, len(Regions))]))
print("--- uv robots ---")
print(uv_robots)

# small uv robots
small_uv_robots = []
for i in range(SMALL_UV_NUMS):
    small_uv_robots.append(
        Robot(r_id_cnt := r_id_cnt + 1, small_uv_category, Regions[random.randrange(0, len(Regions))]))
print("--- small uv robots ---")
print(small_uv_robots)
print()

# register robots
all_robots = []
all_robots.extend(uav_robots)
all_robots.extend(uv_robots)
all_robots.extend(small_uv_robots)
for r in all_robots:
    multi_agents_system.registerRobot(r)

# tasks
tasks = []
for i in range(TASK_NUMS):
    reg = Regions[random.randrange(len(Regions))]
    sp = reg.center
    ep = Point(sp[0] + grid_granularity / 2, sp[1] + grid_granularity / 2)
    task_area = SenseArea(sp, ep)
    st = random.randrange(0, time_long)
    # task_time = TimeRange(st, st + 60 * 60)
    task_time = TimeRange(0, time_long)
    tasks.append(Task(i, sensors[0], task_area, task_time))
print(tasks)

# publish tasks
for task in tasks:
    multi_agents_system.publishTask(task)
    # print(task.TR)

# real word
real_word = RealWorld(len(Regions), (0, 0, 0), (1, 1, 1))


# sim
physical_robots = {r.id: physicalRobot(r) for r in all_robots}
sim_sys = Simulator(physical_robots, multi_agents_system, real_word)

sim_sys.run(time_long)

pltMASys(multi_agents_system)
