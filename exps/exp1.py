from exper import exp
from MASys import RobotOrientAlgorithm, RandomAlgorithm

# experiment parameters
RANDOM_SEED = 0
UAV_NUMS = 5
UV_NUMS = 4
SMALL_UV_NUMS = 4
TASK_NUMS = 60

area_len = 2_000
grid_granularity = 100
time_long = 86400
time_granularity = 3600
CANNOT_SENSE_RETE = (0.1, 0.2, 0.1)

robot_algorithm = RobotOrientAlgorithm

print()
print("="*200)
print()

cov, dist = exp(
    RANDOM_SEED,
    area_len,
    time_long,
    grid_granularity,
    time_granularity,
    TASK_NUMS,
    UV_NUMS,
    UAV_NUMS,
    SMALL_UV_NUMS,
    CANNOT_SENSE_RETE,
    self_repair=False,
    algorithmClass=robot_algorithm,
    # dump_path='./senseMapData',
    # map_file='./senseMapData/4.mapdata'
    no_sense_map=True
)
print(cov, dist)

print()
print("="*120)
print()

# cov, dist = exp(
#     RANDOM_SEED,
#     area_len,
#     time_long,
#     grid_granularity,
#     time_granularity,
#     TASK_NUMS,
#     UV_NUMS,
#     UAV_NUMS,
#     SMALL_UV_NUMS,
#     CANNOT_SENSE_RETE,
#     self_repair=False,
#     algorithmClass=RandomAlgorithm,
# )
# print(cov, dist)
