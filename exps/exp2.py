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
CANNOT_SENSE_RETE = (0.05, 0.1, 0)

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
    self_repair=True,
    algorithmClass=RobotOrientAlgorithm,
    # algorithmClass=RandomAlgorithm,
)
print(cov, dist)
