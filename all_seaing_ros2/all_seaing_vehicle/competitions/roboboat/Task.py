class Task:
    def get_name(self):
        return "uhh... this class was supposed to be abstract lmao"
    def start(self):
        pass
    def update(self):
        pass
    def check_finished(self):
        return False
    def end(self):
        pass
    def get_next(self):
        return False
    def receive_odometry(self, _):
        pass
    def receive_buoys(self, _):
        pass