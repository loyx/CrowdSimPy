import queue

from senseArea import *


async def physicalRobot():
    pass


Event = collections.namedtuple("Event", "time proc")


class Simulator:

    def __init__(self):
        self.events = queue.PriorityQueue()

    def run(self):
        pass


def main():
    pass


if __name__ == '__main__':
    main()
