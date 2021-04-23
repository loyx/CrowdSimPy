
class Sensor:
    SC = set()

    def __init__(self, sid, category, accuracy, s_range):
        if sid in Sensor.SC:
            raise ValueError(f"sensor {sid} already existed")
        self.id = sid
        Sensor.SC.add(sid)
        self.category = category
        self.accuracy = accuracy
        self.range = s_range

    def __repr__(self):
        return "Sensor({}: {}, {}, {})".format(self.id, self.category, self.accuracy, self.range)

    def __eq__(self, other):
        return


def SensorGenerator():
    sensors = [
        Sensor(1, "摄像头", 1200, 100),
        Sensor(2, "可燃气体传感器", 0, 0),
        Sensor(3, "硫化气体传感器", 0, 0),
        Sensor(4, "PM2.5传感器", 0, 0)
    ]
    return sensors


if __name__ == '__main__':
    print(SensorGenerator())
    print(Sensor.SC)
