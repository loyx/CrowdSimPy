
class FeedBack:

    STATUS = {
        0: "nothing",
        1: "need repair"
    }

    def __init__(self, status_code, robots=None):
        if status_code not in FeedBack.STATUS:
            raise ValueError(f"no status code {status_code}")
        self.status_code = status_code
        self.robots = robots


class Message:

    STATUS = {
        0: "robot submit tasks",
        1: "robot is damaged",
        2: "robot cannot complete the plan",
        3: "robot cannot sense this reg",
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
