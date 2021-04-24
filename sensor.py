
class Sensor:
    SC = set()

    def __init__(self, sid, category, accuracy, a_unit, s_range, r_unit):
        if sid in Sensor.SC:
            raise ValueError(f"sensor {sid} already existed")
        self.id = sid
        Sensor.SC.add(sid)
        self.category = category
        self.accuracy = accuracy
        self.a_unit = a_unit
        self.range = s_range
        self.r_unit = r_unit

    def __repr__(self):
        return "Sensor(id:{}: category:{}, acc:{}{}, range:{}{})"\
            .format(self.id, self.category, self.accuracy, self.a_unit, self.range, self.r_unit)

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
