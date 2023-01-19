class Subtask:
    SUCCESS = 0
    ARGUMENT_ERROR = 1
    SERVICE_ERROR = 2
    TOPIC_ERROR = 3
    ACTION_ERROR = 4

    def __init__(self, name, priority=0):
        self.name = name
        self.priority = priority

    def execute(self):
        pass