
class Sensor:
    SC = set()

    def __init__(self, category, accuracy, s_range):
        self.category = category
        if category not in self.SC:
            self.SC.add(category)

        self.accuracy = accuracy
        self.range = s_range

    def __repr__(self):
        return "Sensor({}, {}, {})".format(self.category, self.accuracy, self.range)


def SensorGenerator():
    sensors = [
        Sensor("摄像头", 1200, 100),
        Sensor("可燃气体传感器", 0, 0),
        Sensor("硫化气体传感器", 0, 0),
        Sensor("PM2.5传感器", 0, 0)
    ]
    return sensors


if __name__ == '__main__':
    print(SensorGenerator())
    print(Sensor.SC)
