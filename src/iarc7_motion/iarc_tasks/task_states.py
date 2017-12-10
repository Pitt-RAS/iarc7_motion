#!/usr/bin/env python

class TaskState(object):
    def __init__(self):
        pass

class TaskRunning(TaskState):
    def __init__(self):
        pass

class TaskDone(TaskState):
    def __init__(self):
        pass

class TaskCanceled(TaskState):
    def __init__(self):
        pass

class TaskAborted(TaskState):
    def __init__(self, msg=None):
        self.msg = msg

class TaskFailed(TaskState):
    def __init__(self, msg=None):
        self.msg = msg

