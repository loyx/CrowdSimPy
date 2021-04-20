
class Message:

    STATUS = {
        0: "robot submit tasks",
        1: "robot is damaged",
        2: "robot cannot complete the plan"
    }

    def __init__(self, sc, r_id, rc, reg, rt):
        if sc not in Message.STATUS:
            raise ValueError(f"no status code {sc}")
        self.status_code = sc
        self.robot_id = r_id
        self.robot_category = rc
        self.region = reg
        self.real_time = rt

    def __repr__(self):
        return f"Message({self.real_time:.2f}, {Message.STATUS[self.status_code]})"
