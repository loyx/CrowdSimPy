
class Sensor:

    def __init__(self, sid, category, accuracy, a_unit, s_range, r_unit):
        self.id = sid
        self.category = category
        self.accuracy = accuracy
        self.a_unit = a_unit
        self.range = s_range
        self.r_unit = r_unit

    def __repr__(self):
        return "Sensor(id:{}: category:{}, acc:{}{}, range:{}{})"\
            .format(self.id, self.category, self.accuracy, self.a_unit, self.range, self.r_unit)

    def __hash__(self):
        return hash(self.id)
